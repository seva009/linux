// SPDX-License-Identifier: GPL-2.0-only
/*
 * Synaptics TouchComm (TCM) I2C touchscreen driver.
 *
 * Covers TouchComm generation-1 controllers that boot into application mode out
 * of their own flash, i.e. the ones that need no host firmware download at
 * probe time. The Synaptics S3908 fitted to the OnePlus 9RT (oneplus,martini)
 * is one of those: its vendor driver
 * (drivers/input/touchscreen/oplus_touchscreen_v2/Synaptics/Syna_tcm_oncell)
 * carries no zeroflash/HDL module at all, so a reset is enough to get a running
 * application.
 *
 * Protocol, as implemented by that vendor driver:
 *
 *   A command is one plain I2C write of
 *
 *      [ cmd | len_lo | len_hi | payload... ]
 *
 *   A message -- both command responses and asynchronous reports -- is one
 *   plain I2C read of
 *
 *      [ 0xa5 | code | len_lo | len_hi | payload... | 0x5a ]
 *
 *   Over-reading is fine and is how the vendor driver works too ("predictive
 *   reading"): the controller pads with 0x5a. Reads are capped at 256 bytes,
 *   and a message longer than that is retrieved with follow-up reads that each
 *   come back as [ 0xa5 | 0x03 | data... ].
 *
 *   code <= 0x0f (and 0xff) is a status, so the message is a command response;
 *   code >= 0x10 is a report id.
 *
 * The layout of a touch report is not fixed: the controller carries a "touch
 * report config" -- a list of (opcode, bit width) pairs with loop markers --
 * that says how the report's bit stream is packed, and syna_parse_touch_report()
 * is an interpreter for it. So the driver adapts to whatever the firmware was
 * configured with instead of hardcoding a layout.
 *
 * Deliberately not implemented: firmware update, gesture wakeup, production
 * tests, and suspend/resume. All commands are issued from probe(), before the
 * interrupt is requested, which is why none of this needs locking -- adding
 * PM means adding that.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#define SYNA_MESSAGE_MARKER		0xa5
#define SYNA_HEADER_SIZE		4

/*
 * Read length limit of the controller (RD_CHUNK_SIZE in the vendor driver).
 * One read may not exceed this, including the header.
 */
#define SYNA_RD_CHUNK_SIZE		256

/* Commands. */
#define SYNA_CMD_IDENTIFY		0x02
#define SYNA_CMD_ENABLE_REPORT		0x05
#define SYNA_CMD_GET_APP_INFO		0x20
#define SYNA_CMD_GET_TOUCH_REPORT_CONFIG 0x25
#define SYNA_CMD_SET_TOUCH_REPORT_CONFIG 0x26

/* Status codes, i.e. header codes of a command response. */
#define SYNA_STATUS_IDLE		0x00
#define SYNA_STATUS_OK			0x01
#define SYNA_STATUS_BUSY		0x02
#define SYNA_STATUS_CONTINUED_READ	0x03
#define SYNA_STATUS_ERROR		0x0f
#define SYNA_STATUS_INVALID		0xff

/* Report ids. Anything >= SYNA_REPORT_IDENTIFY is a report, not a status. */
#define SYNA_REPORT_IDENTIFY		0x10
#define SYNA_REPORT_TOUCH		0x11

#define SYNA_MODE_APPLICATION		0x01

/* Opcodes of the touch report config. */
enum syna_touch_report_code {
	SYNA_TOUCH_END				= 0,
	SYNA_TOUCH_FOREACH_ACTIVE_OBJECT	= 1,
	SYNA_TOUCH_FOREACH_OBJECT		= 2,
	SYNA_TOUCH_FOREACH_END			= 3,
	SYNA_TOUCH_PAD_TO_NEXT_BYTE		= 4,
	SYNA_TOUCH_TIMESTAMP			= 5,
	SYNA_TOUCH_OBJECT_N_INDEX		= 6,
	SYNA_TOUCH_OBJECT_N_CLASSIFICATION	= 7,
	SYNA_TOUCH_OBJECT_N_X_POSITION		= 8,
	SYNA_TOUCH_OBJECT_N_Y_POSITION		= 9,
	SYNA_TOUCH_OBJECT_N_Z			= 10,
	SYNA_TOUCH_OBJECT_N_X_WIDTH		= 11,
	SYNA_TOUCH_OBJECT_N_Y_WIDTH		= 12,
	SYNA_TOUCH_NUM_OF_ACTIVE_OBJECTS	= 24,
};

/* Object classification. Anything other than LIFT counts as a contact. */
#define SYNA_OBJECT_LIFT		0

#define SYNA_MAX_OBJECTS		16
#define SYNA_MAX_CONFIG_SIZE		128
#define SYNA_MAX_MESSAGE_SIZE		1024

/* Vendor driver: POWEWRUP_TO_RESET_TIME and RESET_TO_NORMAL_TIME. */
#define SYNA_POWERUP_TO_RESET_MS	10
#define SYNA_RESET_TO_NORMAL_MS   1000

#define SYNA_RESPONSE_TIMEOUT_MS	500

struct syna_object {
	u32 status;
	u32 x;
	u32 y;
	u32 z;
	u32 wx;
	u32 wy;
};

struct syna_tcm {
	struct i2c_client *client;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[2];

	/* Scratch for one I2C read, and the message assembled out of them. */
	u8 rxbuf[SYNA_RD_CHUNK_SIZE];
	u8 payload[SYNA_MAX_MESSAGE_SIZE];
	unsigned int payload_len;
	u8 code;

	u8 cmdbuf[SYNA_MAX_CONFIG_SIZE + 3];

	u8 config[SYNA_MAX_CONFIG_SIZE];
	unsigned int config_len;

	unsigned int max_objects;
	struct syna_object objects[SYNA_MAX_OBJECTS];
};

/*
 * Receive one whole message. On success ts->code holds the header code and the
 * first ts->payload_len bytes of ts->payload hold its payload.
 */
static int syna_recv_message(struct syna_tcm *ts)
{
	struct device *dev = &ts->client->dev;
	unsigned int total, copied, remaining;
	int ret;

	ret = i2c_master_recv(ts->client, ts->rxbuf, SYNA_RD_CHUNK_SIZE);
	if (ret < 0) {
		dev_info(dev, "i2c_master_recv failed: %d\n", ret);
		return ret;
	}

	// dev_info(dev, "rx[0..7]: %02x %02x %02x %02x %02x %02x %02x %02x (ret=%d)\n",
	// 	 ts->rxbuf[0], ts->rxbuf[1], ts->rxbuf[2], ts->rxbuf[3],
	//   ts->rxbuf[4], ts->rxbuf[5], ts->rxbuf[6], ts->rxbuf[7], ret);

	if (ts->rxbuf[0] != SYNA_MESSAGE_MARKER) {
		dev_dbg(dev, "bad message marker 0x%02x\n", ts->rxbuf[0]);
		return -EPROTO;
	}

	ts->code = ts->rxbuf[1];
	ts->payload_len = get_unaligned_le16(&ts->rxbuf[2]);

	/*
	 * An idle or busy controller answers with a header and nothing behind
	 * it, whatever the length field happens to say.
	 */
	if (ts->code == SYNA_STATUS_IDLE || ts->code == SYNA_STATUS_BUSY ||
	    ts->code == SYNA_STATUS_CONTINUED_READ) {
		ts->payload_len = 0;
		return 0;
	}

	if (ts->payload_len > sizeof(ts->payload)) {
		dev_err(dev, "message payload of %u bytes is too long\n",
			ts->payload_len);
		return -EMSGSIZE;
	}

	/* Header, payload, and the trailing 0x5a padding byte. */
	total = SYNA_HEADER_SIZE + ts->payload_len + 1;
	copied = min(ts->payload_len, SYNA_RD_CHUNK_SIZE - SYNA_HEADER_SIZE);
	memcpy(ts->payload, &ts->rxbuf[SYNA_HEADER_SIZE], copied);

	if (total <= SYNA_RD_CHUNK_SIZE)
		return 0;

	/*
	 * Continued read: the rest arrives in chunks that repeat the marker and
	 * carry code 0x03, so only SYNA_RD_CHUNK_SIZE - 2 bytes of each one are
	 * payload.
	 */
	remaining = ts->payload_len - copied;
	while (remaining) {
		unsigned int chunk = min(remaining, SYNA_RD_CHUNK_SIZE - 2);

		ret = i2c_master_recv(ts->client, ts->rxbuf, chunk + 2);
		if (ret < 0)
			return ret;

		if (ts->rxbuf[0] != SYNA_MESSAGE_MARKER ||
		    ts->rxbuf[1] != SYNA_STATUS_CONTINUED_READ) {
			dev_err(dev, "bad continued-read header 0x%02x 0x%02x\n",
				ts->rxbuf[0], ts->rxbuf[1]);
			return -EPROTO;
		}

		memcpy(&ts->payload[copied], &ts->rxbuf[2], chunk);
		copied += chunk;
		remaining -= chunk;
	}

	return 0;
}

/*
 * Issue a command and collect its response into ts->payload. Reports that
 * arrive while waiting are dropped: this only ever runs from probe(), where
 * there is no input device to report them to yet.
 */
static int syna_exec_command(struct syna_tcm *ts, u8 cmd,
			     const u8 *payload, unsigned int len)
{
	struct device *dev = &ts->client->dev;
	unsigned long deadline;
	u8 *buf = ts->cmdbuf;
	int ret;

	if (len + 3 > sizeof(ts->cmdbuf))
		return -EINVAL;

	buf[0] = cmd;
	put_unaligned_le16(len, &buf[1]);
	if (len)
		memcpy(&buf[3], payload, len);

	dev_info(dev, "sending cmd 0x%02x, %u bytes\n", cmd, len + 3);
	ret = i2c_master_send(ts->client, buf, len + 3);
	dev_info(dev, "i2c_master_send returned %d\n", ret);
	if (ret < 0) {
		dev_err(dev, "failed to write command 0x%02x: %d\n", cmd, ret);
		return ret;
	}

	deadline = jiffies + msecs_to_jiffies(SYNA_RESPONSE_TIMEOUT_MS);
	do {
		usleep_range(15000, 20000);

		ret = syna_recv_message(ts);
		if (ret == -EPROTO)
			continue;	/* controller not talking yet */
		if (ret < 0)
			return ret;

		if (ts->code == SYNA_REPORT_IDENTIFY && cmd == SYNA_CMD_IDENTIFY)
			return 0;

		if (ts->code >= SYNA_REPORT_IDENTIFY)
			continue;	/* a report, not our response */

		switch (ts->code) {
		case SYNA_STATUS_OK:
			return 0;
		case SYNA_STATUS_IDLE:
		case SYNA_STATUS_BUSY:
			continue;
		default:
			dev_err(dev, "command 0x%02x failed with status 0x%02x\n",
				cmd, ts->code);
			return -EIO;
		}
	} while (time_before(jiffies, deadline));

	dev_err(dev, "command 0x%02x timed out\n", cmd);
	return -ETIMEDOUT;
}

/* Extract a big field of @bits from the report's bit stream at @offset. */
static u32 syna_get_bits(const u8 *buf, unsigned int buf_len,
			 unsigned int offset, unsigned int bits)
{
	unsigned int byte = offset / 8;
	unsigned int bit = offset % 8;
	unsigned int done = 0;
	u32 out = 0;

	if (!bits || bits > 32 || offset + bits > buf_len * 8)
		return 0;

	while (done < bits) {
		unsigned int avail = 8 - bit;
		unsigned int take = min(avail, bits - done);
		u8 val = (buf[byte] >> bit) & (0xff >> (8 - take));

		out |= (u32)val << done;
		done += take;
		bit = 0;
		byte++;
	}

	return out;
}

/*
 * Walk the touch report config and unpack the report accordingly. Mirrors
 * syna_parse_report() in the vendor driver.
 */
static int syna_parse_touch_report(struct syna_tcm *ts)
{
	const u8 *cfg = ts->config;
	unsigned int idx = 0, loop_start = 0;
	unsigned int offset = 0;	/* bit offset into the payload */
	unsigned int obj = 0;
	unsigned int seen = 0, active = 0;
	bool active_only = false, have_active_count = false;
	bool done = false;

	memset(ts->objects, 0, sizeof(ts->objects));

	while (idx < ts->config_len && !done) {
		u8 code = cfg[idx++];
		unsigned int bits;
		u32 data;

		switch (code) {
		case SYNA_TOUCH_END:
			done = true;
			continue;

		case SYNA_TOUCH_FOREACH_ACTIVE_OBJECT:
		case SYNA_TOUCH_FOREACH_OBJECT:
			active_only = code == SYNA_TOUCH_FOREACH_ACTIVE_OBJECT;
			loop_start = idx;
			obj = 0;
			continue;

		case SYNA_TOUCH_FOREACH_END:
			if (!active_only) {
				if (++obj < ts->max_objects)
					idx = loop_start;
			} else if (have_active_count) {
				if (++seen < active)
					idx = loop_start;
			} else if (offset < ts->payload_len * 8) {
				/*
				 * No explicit contact count in this config, so
				 * the payload length is the count: keep going
				 * while there are bits left. This is the case
				 * for the config the vendor driver installs.
				 */
				idx = loop_start;
			}
			continue;

		case SYNA_TOUCH_PAD_TO_NEXT_BYTE:
			offset = ALIGN(offset, 8);
			continue;
		}

		/* Everything else is (opcode, bit width). */
		if (idx >= ts->config_len)
			break;

		bits = cfg[idx++];
		data = syna_get_bits(ts->payload, ts->payload_len, offset, bits);
		offset += bits;

		switch (code) {
		case SYNA_TOUCH_OBJECT_N_INDEX:
			if (data >= ts->max_objects) {
				dev_dbg(&ts->client->dev,
					"object index %u out of range\n", data);
				return -ERANGE;
			}
			obj = data;
			break;
		case SYNA_TOUCH_NUM_OF_ACTIVE_OBJECTS:
			active = data;
			have_active_count = true;
			if (!active)
				done = true;
			break;
		case SYNA_TOUCH_OBJECT_N_CLASSIFICATION:
			ts->objects[obj].status = data;
			break;
		case SYNA_TOUCH_OBJECT_N_X_POSITION:
			ts->objects[obj].x = data;
			break;
		case SYNA_TOUCH_OBJECT_N_Y_POSITION:
			ts->objects[obj].y = data;
			break;
		case SYNA_TOUCH_OBJECT_N_Z:
			ts->objects[obj].z = data;
			break;
		case SYNA_TOUCH_OBJECT_N_X_WIDTH:
			ts->objects[obj].wx = data;
			break;
		case SYNA_TOUCH_OBJECT_N_Y_WIDTH:
			ts->objects[obj].wy = data;
			break;
		default:
			/* Timestamp, gestures, grip info, tuning: skipped. */
			break;
		}
	}

	return 0;
}

static void syna_report_touch(struct syna_tcm *ts)
{
	unsigned int i;

	for (i = 0; i < ts->max_objects; i++) {
		struct syna_object *obj = &ts->objects[i];
		bool down = obj->status != SYNA_OBJECT_LIFT;

		if (down)
			dev_dbg(&ts->client->dev,
				"slot %u: %u,%u z=%u w=%ux%u class=%u\n",
				i, obj->x, obj->y, obj->z, obj->wx, obj->wy,
				obj->status);

		input_mt_slot(ts->input, i);
		if (!input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, down))
			continue;

		touchscreen_report_pos(ts->input, &ts->prop, obj->x, obj->y, true);
		input_report_abs(ts->input, ABS_MT_PRESSURE, obj->z);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
				 max(obj->wx, obj->wy));
		input_report_abs(ts->input, ABS_MT_TOUCH_MINOR,
				 min(obj->wx, obj->wy));
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static irqreturn_t syna_irq(int irq, void *data)
{
	struct syna_tcm *ts = data;

	if (syna_recv_message(ts) < 0)
		return IRQ_NONE;

	if (ts->code != SYNA_REPORT_TOUCH)
		return IRQ_HANDLED;

	if (syna_parse_touch_report(ts) == 0)
		syna_report_touch(ts);

	return IRQ_HANDLED;
}

static int syna_power_up(struct syna_tcm *ts)
{
	int ret;

	/* Held in reset across power-up; the DT flag makes this drive low. */
	gpiod_set_value_cansleep(ts->reset_gpio, 1);

	ret = regulator_bulk_enable(ARRAY_SIZE(ts->supplies), ts->supplies);
	if (ret)
		return ret;

	msleep(SYNA_POWERUP_TO_RESET_MS);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(SYNA_RESET_TO_NORMAL_MS);

	return 0;
}

static void syna_power_down(void *data)
{
	struct syna_tcm *ts = data;

	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ts->supplies), ts->supplies);
}

/*
 * A touch report config we can definitely parse, installed only if the one the
 * firmware came up with carries no coordinates. Same as the vendor driver's,
 * minus its oplus-specific grip-info field.
 */
static const u8 syna_default_config[] = {
	SYNA_TOUCH_FOREACH_ACTIVE_OBJECT,
	SYNA_TOUCH_OBJECT_N_INDEX,		4,
	SYNA_TOUCH_OBJECT_N_CLASSIFICATION,	4,
	SYNA_TOUCH_OBJECT_N_X_POSITION,		16,
	SYNA_TOUCH_OBJECT_N_Y_POSITION,		16,
	SYNA_TOUCH_OBJECT_N_X_WIDTH,		12,
	SYNA_TOUCH_OBJECT_N_Y_WIDTH,		12,
	SYNA_TOUCH_FOREACH_END,
	SYNA_TOUCH_END,
};

static bool syna_config_has_coords(const u8 *cfg, unsigned int len)
{
	bool x = false, y = false;
	unsigned int i;

	for (i = 0; i < len; i++) {
		switch (cfg[i]) {
		case SYNA_TOUCH_END:
			return x && y;
		case SYNA_TOUCH_FOREACH_ACTIVE_OBJECT:
		case SYNA_TOUCH_FOREACH_OBJECT:
		case SYNA_TOUCH_FOREACH_END:
		case SYNA_TOUCH_PAD_TO_NEXT_BYTE:
			break;
		default:
			if (cfg[i] == SYNA_TOUCH_OBJECT_N_X_POSITION)
				x = true;
			else if (cfg[i] == SYNA_TOUCH_OBJECT_N_Y_POSITION)
				y = true;
			i++;	/* skip the bit width */
			break;
		}
	}

	return x && y;
}

static int syna_read_touch_report_config(struct syna_tcm *ts)
{
	struct device *dev = &ts->client->dev;
	int ret;

	ret = syna_exec_command(ts, SYNA_CMD_GET_TOUCH_REPORT_CONFIG, NULL, 0);
	if (ret)
		return ret;

	if (!ts->payload_len || ts->payload_len > sizeof(ts->config)) {
		dev_err(dev, "implausible touch report config size %u\n",
			ts->payload_len);
		return -EPROTO;
	}

	ts->config_len = ts->payload_len;
	memcpy(ts->config, ts->payload, ts->config_len);

	dev_info(dev, "touch report config: %*ph\n", ts->config_len, ts->config);

	return 0;
}

static int syna_setup(struct syna_tcm *ts)
{
	struct device *dev = &ts->client->dev;
	unsigned int max_x, max_y;
	u8 report = SYNA_REPORT_TOUCH;
	int ret;

	ret = syna_exec_command(ts, SYNA_CMD_IDENTIFY, NULL, 0);
	if (ret)
		return ret;

	/* struct syna_tcm_identification: version, mode, part_number[16]. */
	if (ts->payload_len < 18)
		return -EPROTO;

	dev_info(dev, "TouchComm v%u, mode 0x%02x, part %.16s\n",
		 ts->payload[0], ts->payload[1], &ts->payload[2]);

	if (ts->payload[1] != SYNA_MODE_APPLICATION) {
		dev_err(dev,
			"controller is in mode 0x%02x, not application mode; this driver cannot download firmware\n",
			ts->payload[1]);
		return -ENODEV;
	}

	ret = syna_exec_command(ts, SYNA_CMD_GET_APP_INFO, NULL, 0);
	if (ret)
		return ret;

	// print_hex_dump(KERN_INFO, "syna app_info: ", DUMP_PREFIX_OFFSET,
	// 	       16, 1, ts->payload, ts->payload_len, false);

	/* struct syna_tcm_app_info: max_x at 38, max_y at 40, max_objects at 42. */
	if (ts->payload_len < 44)
		return -EPROTO;

	max_x = get_unaligned_le16(&ts->payload[38]);
	max_y = get_unaligned_le16(&ts->payload[40]);
	ts->max_objects = get_unaligned_le16(&ts->payload[42]);

	if (!max_x || !max_y || !ts->max_objects) {
		dev_warn(dev, "app_info gave x=%u y=%u objects=%u, falling back to defaults\n",
			 max_x, max_y, ts->max_objects);
		max_x = 3216;      /* panel native width, from your build.sh oplus20031 overlay name */
		max_y = 1440;      /* panel native height */
		ts->max_objects = 10;
	}

	if (ts->max_objects > SYNA_MAX_OBJECTS) {
		dev_warn(dev, "clamping %u objects to %u\n",
			 ts->max_objects, SYNA_MAX_OBJECTS);
		ts->max_objects = SYNA_MAX_OBJECTS;
	}

	dev_info(dev, "%ux%u, %u touch objects\n", max_x + 1, max_y + 1,
		 ts->max_objects);

	ret = syna_read_touch_report_config(ts);
	if (ret)
		return ret;

	if (!syna_config_has_coords(ts->config, ts->config_len)) {
		dev_warn(dev,
			 "firmware touch report config carries no coordinates, installing our own\n");

		ret = syna_exec_command(ts, SYNA_CMD_SET_TOUCH_REPORT_CONFIG,
					syna_default_config,
					sizeof(syna_default_config));
		if (ret)
			return ret;

		ret = syna_read_touch_report_config(ts);
		if (ret)
			return ret;

		if (!syna_config_has_coords(ts->config, ts->config_len)) {
			dev_err(dev, "controller kept a config we cannot parse\n");
			return -ENODEV;
		}
	}

	ret = syna_exec_command(ts, SYNA_CMD_ENABLE_REPORT, &report, 1);
	if (ret)
		return ret;

	ts->input->name = "Synaptics TouchComm Touchscreen";
	ts->input->id.bustype = BUS_I2C;

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 4095, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MINOR, 0, 4095, 0, 0);

	/* Lets a board override the axes with touchscreen-inverted-* etc. */
	touchscreen_parse_properties(ts->input, true, &ts->prop);

	return input_mt_init_slots(ts->input, ts->max_objects,
				   INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
}

static int syna_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct syna_tcm *ts;
	int ret;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;

	/* vdd is the 1.8 V logic/IO rail, avdd the ~3 V analog one. */
	ts->supplies[0].supply = "vdd";
	ts->supplies[1].supply = "avdd";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ts->supplies),
				      ts->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	ts->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset GPIO\n");

	ret = syna_power_up(ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power up\n");

	ret = devm_add_action_or_reset(dev, syna_power_down, ts);
	if (ret)
		return ret;

	ret = syna_setup(ts);
	if (ret)
		return ret;

	ret = input_register_device(ts->input);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register input device\n");

	/*
	 * Last: syna_exec_command() and syna_irq() both drive the same receive
	 * path with no lock between them, which is only safe because every
	 * command has already been issued by now.
	 *
	 * The line is level-triggered and active low -- the controller holds it
	 * down until the pending message has been read -- so the handler must
	 * be threaded and one-shot.
	 */
	ret = devm_request_threaded_irq(dev, client->irq, NULL, syna_irq,
					IRQF_ONESHOT, "synaptics-tcm", ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ %d\n",
				     client->irq);

	return 0;
}

static const struct i2c_device_id syna_tcm_i2c_id[] = {
	{ "synaptics-tcm" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, syna_tcm_i2c_id);

static const struct of_device_id syna_tcm_of_match[] = {
	{ .compatible = "syna,s3908" },
	{ }
};
MODULE_DEVICE_TABLE(of, syna_tcm_of_match);

static struct i2c_driver syna_tcm_i2c_driver = {
	.driver = {
		.name = "synaptics-tcm-i2c",
		.of_match_table = syna_tcm_of_match,
	},
	.probe = syna_probe,
	.id_table = syna_tcm_i2c_id,
};
module_i2c_driver(syna_tcm_i2c_driver);

MODULE_DESCRIPTION("Synaptics TouchComm I2C touchscreen driver");
MODULE_LICENSE("GPL");
