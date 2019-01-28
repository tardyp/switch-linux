// SPDX-License-Identifier: GPL-2.0+
/*
 * HID driver for Nintendo Switch Joy-Cons and Pro Controllers
 *
 * Copyright (c) 2019 Daniel J. Ogorchock <djogorchock@gmail.com>
 *
 * The following resources/projects were referenced for this driver:
 *   https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering
 *   https://gitlab.com/pjranki/joycon-linux-kernel (Peter Rankin)
 *   https://github.com/FrotBot/SwitchProConLinuxUSB
 *   https://github.com/MTCKC/ProconXInput
 *   hid-wiimote kernel hid driver
 *   hid-logitech-hidpp driver
 *   hid-sony driver
 *
 * This driver supports the Nintendo Switch Joy-Cons and Pro Controllers. The
 * Pro Controllers can either be used over USB or Bluetooth.
 *
 * The driver will retrieve the factory calibration info from the controllers,
 * so little to no user calibration should be required.
 *
 */

#include "hid-ids.h"
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/spinlock.h>

/*
 * Reference the url below for the following HID report defines:
 * https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering
 */

/* Output Reports */
#define SC_OUTPUT_RUMBLE_AND_SUBCMD	0x01
#define SC_OUTPUT_FW_UPDATE_PKT		0x03
#define SC_OUTPUT_RUMBLE_ONLY		0x10
#define SC_OUTPUT_MCU_DATA		0x11
#define SC_OUTPUT_USB_CMD		0x80

/* Subcommand IDs */
#define SC_SUBCMD_STATE			0x00
#define SC_SUBCMD_MANUAL_BT_PAIRING	0x01
#define SC_SUBCMD_REQ_DEV_INFO		0x02
#define SC_SUBCMD_SET_REPORT_MODE	0x03
#define SC_SUBCMD_TRIGGERS_ELAPSED	0x04
#define SC_SUBCMD_GET_PAGE_LIST_STATE	0x05
#define SC_SUBCMD_SET_HCI_STATE		0x06
#define SC_SUBCMD_RESET_PAIRING_INFO	0x07
#define SC_SUBCMD_LOW_POWER_MODE	0x08
#define SC_SUBCMD_SPI_FLASH_READ	0x10
#define SC_SUBCMD_SPI_FLASH_WRITE	0x11
#define SC_SUBCMD_RESET_MCU		0x20
#define SC_SUBCMD_SET_MCU_CONFIG	0x21
#define SC_SUBCMD_SET_MCU_STATE		0x22
#define SC_SUBCMD_SET_PLAYER_LIGHTS	0x30
#define SC_SUBCMD_GET_PLAYER_LIGHTS	0x31
#define SC_SUBCMD_SET_HOME_LIGHT	0x38
#define SC_SUBCMD_ENABLE_IMU		0x40
#define SC_SUBCMD_SET_IMU_SENSITIVITY	0x41
#define SC_SUBCMD_WRITE_IMU_REG		0x42
#define SC_SUBCMD_READ_IMU_REG		0x43
#define SC_SUBCMD_ENABLE_VIBRATION	0x48
#define SC_SUBCMD_GET_REGULATED_VOLTAGE	0x50

/* Input Reports */
#define SC_INPUT_BUTTON_EVENT		0x3F
#define SC_INPUT_SUBCMD_REPLY		0x21
#define SC_INPUT_IMU_DATA		0x30
#define SC_INPUT_MCU_DATA		0x31
#define SC_INPUT_USB_RESPONSE		0x81

/* Feature Reports */
#define SC_FEATURE_LAST_SUBCMD		0x02
#define SC_FEATURE_OTA_FW_UPGRADE	0x70
#define SC_FEATURE_SETUP_MEM_READ	0x71
#define SC_FEATURE_MEM_READ		0x72
#define SC_FEATURE_ERASE_MEM_SECTOR	0x73
#define SC_FEATURE_MEM_WRITE		0x74
#define SC_FEATURE_LAUNCH		0x75

/* USB Commands */
#define SC_USB_CMD_CONN_STATUS		0x01
#define SC_USB_CMD_HANDSHAKE		0x02
#define SC_USB_CMD_BAUDRATE_3M		0x03
#define SC_USB_CMD_NO_TIMEOUT		0x04
#define SC_USB_CMD_EN_TIMEOUT		0x05
#define SC_USB_RESET			0x06
#define SC_USB_PRE_HANDSHAKE		0x91
#define SC_USB_SEND_UART		0x92

/* SPI storage addresses of factory calibration data */
#define SC_CAL_DATA_START		0x603d
#define SC_CAL_DATA_END			0x604e
#define SC_CAL_DATA_SIZE	(SC_CAL_DATA_END - SC_CAL_DATA_START + 1)


/* The raw analog joystick values will be mapped in terms of this magnitude */
#define SC_MAX_STICK_MAG	32767
#define SC_STICK_FUZZ		250
#define SC_STICK_FLAT		500

/* States for controller state machine */
enum switchcon_ctlr_state {
	SWITCHCON_CTLR_STATE_INIT,
	SWITCHCON_CTLR_STATE_READ,
};

struct switchcon_stick_cal {
	s32 max;
	s32 min;
	s32 center;
};

/*
 * All the controller's button values are stored in a u32.
 * They can be accessed with bitwise ANDs.
 */
#define SC_BTN_Y	BIT(0)
#define SC_BTN_X	BIT(1)
#define SC_BTN_B	BIT(2)
#define SC_BTN_A	BIT(3)
#define SC_BTN_SR_R	BIT(4)
#define SC_BTN_SL_R	BIT(5)
#define SC_BTN_R	BIT(6)
#define SC_BTN_ZR	BIT(7)
#define SC_BTN_MINUS	BIT(8)
#define SC_BTN_PLUS	BIT(9)
#define SC_BTN_RSTICK	BIT(10)
#define SC_BTN_LSTICK	BIT(11)
#define SC_BTN_HOME	BIT(12)
#define SC_BTN_CAP	BIT(13) /* capture button */
#define SC_BTN_DOWN	BIT(16)
#define SC_BTN_UP	BIT(17)
#define SC_BTN_RIGHT	BIT(18)
#define SC_BTN_LEFT	BIT(19)
#define SC_BTN_SR_L	BIT(20)
#define SC_BTN_SL_L	BIT(21)
#define SC_BTN_L	BIT(22)
#define SC_BTN_ZL	BIT(23)

enum switchcon_ctlr_type {
	SWITCHCON_CTLR_TYPE_PROCON,
	SWITCHCON_CTLR_TYPE_JOYCON_L,
	SWITCHCON_CTLR_TYPE_JOYCON_R,
};

static const char * const switchcon_input_names[] = {
	"Nintendo Switch Pro Controller",
	"Nintendo Switch Left Joy-Con",
	"Nintendo Switch Right Joy-Con",
};

enum switchcon_msg_type {
	SWITCHCON_MSG_TYPE_NONE,
	SWITCHCON_MSG_TYPE_USB,
	SWITCHCON_MSG_TYPE_SUBCMD,
};

struct switchcon_subcmd_request {
	u8 output_id; /* must be 0x01 for subcommand, 0x10 for rumble only */
	u8 packet_num; /* incremented every send */
	u8 rumble_data[8];
	u8 subcmd_id;
	/* data is here */
} __packed;

/* should pass in pointer to a struct switchcon_subcmd_request */
#define SC_SUBCMD_REQ_GET_DATA(req) \
	((u8 *)(req) + sizeof(struct switchcon_subcmd_request))

struct switchcon_subcmd_reply {
	u8 ack; /* MSB 1 for ACK, 0 for NACK */
	u8 id; /* id of requested subcmd */
	/* data is here, can be up to 35 bytes */
} __packed;

/* should pass in pointer to a struct switchcon_subcmd_reply */
#define SC_SUBCMD_REPLY_GET_DATA(reply) \
	((u8 *)(reply) + sizeof(struct switchcon_subcmd_reply))

struct switchcon_input_report {
	u8 id;
	u8 timer;
	u8 bat_con; /* battery and connection info */
	u8 button_status[3];
	u8 left_stick[3];
	u8 right_stick[3];
	u8 vibrator_report;

	/*
	 * If support for firmware updates, gyroscope data, and/or NFC/IR
	 * are added in the future, this can be swapped for a union.
	 */
	struct switchcon_subcmd_reply reply;
} __packed;

#define SC_MAX_RESP_SIZE (sizeof(struct switchcon_input_report) + 35)
#define SC_NUM_LEDS 4

/* Each physical controller is associated with a switchcon_ctlr struct */
struct switchcon_ctlr {
	struct hid_device *hdev;
	struct input_dev *input;
	struct led_classdev leds[SC_NUM_LEDS];
	enum switchcon_ctlr_type type;
	enum switchcon_ctlr_state ctlr_state;
	spinlock_t lock;

	/* The following members are used for synchronous sends/receives */
	enum switchcon_msg_type msg_type;
	u8 subcmd_num;
	struct mutex output_mutex;
	u8 input_buf[SC_MAX_RESP_SIZE];
	wait_queue_head_t wait;
	bool received_resp;
	u8 usb_ack_match;
	u8 subcmd_ack_match;

	/* factory calibration data */
	struct switchcon_stick_cal left_stick_cal_x;
	struct switchcon_stick_cal left_stick_cal_y;
	struct switchcon_stick_cal right_stick_cal_x;
	struct switchcon_stick_cal right_stick_cal_y;

	/* power supply data */
	struct power_supply *battery;
	struct power_supply_desc battery_desc;
	u8 battery_capacity;
	bool battery_charging;
	bool host_powered;
};

static int __switchcon_hid_send(struct hid_device *hdev, u8 *data, size_t len)
{
	u8 *buf;
	int ret;

	buf = kmemdup(data, len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	ret = hid_hw_output_report(hdev, buf, len);
	kfree(buf);
	if (ret < 0)
		hid_dbg(hdev, "Failed to send output report ret=%d\n", ret);
	return ret;
}

static int switchcon_hid_send_sync(struct switchcon_ctlr *ctlr, u8 *data,
								size_t len)
{
	int ret;

	ret = __switchcon_hid_send(ctlr->hdev, data, len);
	if (ret < 0) {
		memset(ctlr->input_buf, 0, SC_MAX_RESP_SIZE);
		return ret;
	}

	if (!wait_event_timeout(ctlr->wait, ctlr->received_resp, HZ)) {
		hid_dbg(ctlr->hdev, "syncronous send/receive timed out\n");
		memset(ctlr->input_buf, 0, SC_MAX_RESP_SIZE);
		return -ETIMEDOUT;
	}

	ctlr->received_resp = false;
	return 0;
}

static int switchcon_send_usb(struct switchcon_ctlr *ctlr, u8 cmd)
{
	int ret;
	u8 buf[2] = {SC_OUTPUT_USB_CMD};

	buf[1] = cmd;
	ctlr->usb_ack_match = cmd;
	ctlr->msg_type = SWITCHCON_MSG_TYPE_USB;
	ret = switchcon_hid_send_sync(ctlr, buf, sizeof(buf));
	if (ret)
		hid_dbg(ctlr->hdev, "send usb command failed; ret=%d\n", ret);
	return ret;
}

static int switchcon_send_subcmd(struct switchcon_ctlr *ctlr,
				 struct switchcon_subcmd_request *subcmd,
				 size_t data_len)
{
	int ret;

	subcmd->output_id = SC_OUTPUT_RUMBLE_AND_SUBCMD;
	subcmd->packet_num = ctlr->subcmd_num;
	if (++ctlr->subcmd_num > 0xF)
		ctlr->subcmd_num = 0;
	ctlr->subcmd_ack_match = subcmd->subcmd_id;
	ctlr->msg_type = SWITCHCON_MSG_TYPE_SUBCMD;

	ret = switchcon_hid_send_sync(ctlr, (u8 *)subcmd,
				      sizeof(*subcmd) + data_len);
	if (ret)
		hid_dbg(ctlr->hdev, "send subcommand failed; ret=%d\n", ret);
	return ret;
}

/* Supply nibbles for flash and on. Ones correspond to active */
static int switchcon_set_player_leds(struct switchcon_ctlr *ctlr,
						u8 flash, u8 on)
{
	struct switchcon_subcmd_request *req;
	u8 buffer[sizeof(*req) + 1] = { 0 };

	req = (struct switchcon_subcmd_request *)buffer;
	req->subcmd_id = SC_SUBCMD_SET_PLAYER_LIGHTS;
	SC_SUBCMD_REQ_GET_DATA(req)[0] = (flash << 4) | on;

	hid_dbg(ctlr->hdev, "setting player leds\n");
	return switchcon_send_subcmd(ctlr, req, 1);
}

static int switchcon_request_calibration(struct switchcon_ctlr *ctlr)
{
	struct switchcon_subcmd_request *req;
	u8 buffer[sizeof(*req) + 5] = { 0 };
	u8 *data;

	req = (struct switchcon_subcmd_request *)buffer;
	req->subcmd_id = SC_SUBCMD_SPI_FLASH_READ;
	data = SC_SUBCMD_REQ_GET_DATA(req);
	data[0] = 0xFF & SC_CAL_DATA_START;
	data[1] = 0xFF & (SC_CAL_DATA_START >> 8);
	data[2] = 0xFF & (SC_CAL_DATA_START >> 16);
	data[3] = 0xFF & (SC_CAL_DATA_START >> 24);
	data[4] = SC_CAL_DATA_SIZE;

	hid_dbg(ctlr->hdev, "requesting cal data\n");
	return switchcon_send_subcmd(ctlr, req, 5);
}

static int switchcon_set_report_mode(struct switchcon_ctlr *ctlr)
{
	struct switchcon_subcmd_request *req;
	u8 buffer[sizeof(*req) + 1] = { 0 };

	req = (struct switchcon_subcmd_request *)buffer;
	req->subcmd_id = SC_SUBCMD_SET_REPORT_MODE;
	SC_SUBCMD_REQ_GET_DATA(req)[0] = 0x30; /* standard, full report mode */

	hid_dbg(ctlr->hdev, "setting controller report mode\n");
	return switchcon_send_subcmd(ctlr, req, 1);
}

static int switchcon_map_stick_val(struct switchcon_stick_cal *cal, s32 val)
{
	s32 center = cal->center;
	s32 min = cal->min;
	s32 max = cal->max;
	int new_val;

	if (val > center) {
		new_val = (val - center) * SC_MAX_STICK_MAG;
		new_val /= (max - center);
	} else {
		new_val = (center - val) * -SC_MAX_STICK_MAG;
		new_val /= (center - min);
	}
	new_val = clamp(new_val, -SC_MAX_STICK_MAG, SC_MAX_STICK_MAG);
	return new_val;
}

static void switchcon_parse_report(struct switchcon_ctlr *ctlr, u8 *data)
{
	struct input_dev *dev = ctlr->input;
	enum switchcon_ctlr_type type = ctlr->type;
	unsigned long flags;
	u8 tmp;
	u32 btns;

	/* Parse the battery status */
	tmp = data[2];
	spin_lock_irqsave(&ctlr->lock, flags);
	ctlr->host_powered = tmp & BIT(0);
	ctlr->battery_charging = tmp & BIT(4);
	tmp = tmp >> 5;
	switch (tmp) {
	case 0: /* empty */
		ctlr->battery_capacity = 0;
		break;
	case 1: /* critical */
		ctlr->battery_capacity = 1;
		break;
	case 2: /* low */
		ctlr->battery_capacity = 30;
		break;
	case 3: /* medium */
		ctlr->battery_capacity = 60;
		break;
	case 4: /* full */
		ctlr->battery_capacity = 100;
		break;
	default:
		ctlr->battery_capacity = 0;
		hid_warn(ctlr->hdev, "Invalid battery status\n");
		break;
	}
	spin_unlock_irqrestore(&ctlr->lock, flags);

	/* Parse the buttons and sticks */
	btns = hid_field_extract(ctlr->hdev, data + 3, 0, 24);

	if (type == SWITCHCON_CTLR_TYPE_PROCON ||
	    type == SWITCHCON_CTLR_TYPE_JOYCON_L) {
		u16 raw_l_x;
		u16 raw_l_y;
		int s_l_x;
		int s_l_y;

		/* get raw stick values */
		raw_l_x = hid_field_extract(ctlr->hdev, (data + 6), 0, 12);
		raw_l_y = hid_field_extract(ctlr->hdev, (data + 7), 4, 12);
		/* map the stick values */
		s_l_x = switchcon_map_stick_val(&ctlr->left_stick_cal_x,
						raw_l_x);
		s_l_y = -switchcon_map_stick_val(&ctlr->left_stick_cal_y,
						raw_l_y);
		/* report sticks */
		input_report_abs(dev, ABS_X, s_l_x);
		input_report_abs(dev, ABS_Y, s_l_y);

		/* report buttons */
		input_report_key(dev, BTN_TL, btns & SC_BTN_L ||
					      btns & SC_BTN_SL_L);
		input_report_key(dev, BTN_TR, btns & SC_BTN_R ||
					      btns & SC_BTN_SR_L);
		input_report_key(dev, BTN_TL2, btns & SC_BTN_ZL);
		input_report_key(dev, BTN_SELECT, btns & SC_BTN_MINUS);
		input_report_key(dev, BTN_THUMBL, btns & SC_BTN_LSTICK);
		input_report_key(dev, BTN_Z, btns & SC_BTN_CAP);
		input_report_key(dev, BTN_DPAD_DOWN, btns & SC_BTN_DOWN);
		input_report_key(dev, BTN_DPAD_UP, btns & SC_BTN_UP);
		input_report_key(dev, BTN_DPAD_RIGHT, btns & SC_BTN_RIGHT);
		input_report_key(dev, BTN_DPAD_LEFT, btns & SC_BTN_LEFT);
	}
	if (type == SWITCHCON_CTLR_TYPE_PROCON ||
	    type == SWITCHCON_CTLR_TYPE_JOYCON_R) {
		u16 raw_r_x;
		u16 raw_r_y;
		int s_r_x;
		int s_r_y;

		/* get raw stick values */
		raw_r_x = hid_field_extract(ctlr->hdev, (data + 9), 0, 12);
		raw_r_y = hid_field_extract(ctlr->hdev, (data + 10), 4, 12);
		/* map stick values */
		s_r_x = switchcon_map_stick_val(&ctlr->right_stick_cal_x,
						raw_r_x);
		s_r_y = -switchcon_map_stick_val(&ctlr->right_stick_cal_y,
						raw_r_y);
		/* report sticks */
		input_report_abs(dev, ABS_RX, s_r_x);
		input_report_abs(dev, ABS_RY, s_r_y);

		/* report buttons */
		input_report_key(dev, BTN_TL, btns & SC_BTN_L ||
					      btns & SC_BTN_SL_R);
		input_report_key(dev, BTN_TR, btns & SC_BTN_R ||
					      btns & SC_BTN_SR_R);
		input_report_key(dev, BTN_TR2, btns & SC_BTN_ZR);
		input_report_key(dev, BTN_START, btns & SC_BTN_PLUS);
		input_report_key(dev, BTN_THUMBR, btns & SC_BTN_RSTICK);
		input_report_key(dev, BTN_MODE, btns & SC_BTN_HOME);
		input_report_key(dev, BTN_WEST, btns & SC_BTN_Y);
		input_report_key(dev, BTN_NORTH, btns & SC_BTN_X);
		input_report_key(dev, BTN_EAST, btns & SC_BTN_A);
		input_report_key(dev, BTN_SOUTH, btns & SC_BTN_B);
	}

	input_sync(dev);
}


static const unsigned int switchcon_button_inputs[] = {
	BTN_SELECT, BTN_Z, BTN_THUMBL,
	BTN_START, BTN_MODE, BTN_THUMBR,
	BTN_SOUTH, BTN_EAST, BTN_NORTH, BTN_WEST,
	BTN_DPAD_UP, BTN_DPAD_DOWN, BTN_DPAD_LEFT, BTN_DPAD_RIGHT,
	BTN_TL, BTN_TR, BTN_TL2, BTN_TR2,
	0 /* 0 signals end of array */
};

static int switchcon_input_create(struct switchcon_ctlr *ctlr)
{
	struct hid_device *hdev;
	int ret;
	int i;

	hdev = ctlr->hdev;
	ctlr->input = input_allocate_device();
	if (!ctlr->input) {
		ret = -ENOMEM;
		goto err;
	}
	ctlr->input->dev.parent = &hdev->dev;
	ctlr->input->id.bustype = hdev->bus;
	ctlr->input->id.vendor = hdev->vendor;
	ctlr->input->id.product = hdev->product;
	ctlr->input->id.version = hdev->version;
	ctlr->input->name = switchcon_input_names[ctlr->type];
	input_set_drvdata(ctlr->input, ctlr);


	/* set up sticks */
	input_set_abs_params(ctlr->input, ABS_X,
			     -SC_MAX_STICK_MAG, SC_MAX_STICK_MAG,
			     SC_STICK_FUZZ, SC_STICK_FLAT);
	input_set_abs_params(ctlr->input, ABS_Y,
			     -SC_MAX_STICK_MAG, SC_MAX_STICK_MAG,
			     SC_STICK_FUZZ, SC_STICK_FLAT);
	input_set_abs_params(ctlr->input, ABS_RX,
			     -SC_MAX_STICK_MAG, SC_MAX_STICK_MAG,
			     SC_STICK_FUZZ, SC_STICK_FLAT);
	input_set_abs_params(ctlr->input, ABS_RY,
			     -SC_MAX_STICK_MAG, SC_MAX_STICK_MAG,
			     SC_STICK_FUZZ, SC_STICK_FLAT);
	/* set up buttons */
	for (i = 0; switchcon_button_inputs[i] > 0; i++)
		input_set_capability(ctlr->input, EV_KEY,
				     switchcon_button_inputs[i]);

	ret = input_register_device(ctlr->input);
	if (ret)
		goto err_input;

	return 0;

err_input:
	input_free_device(ctlr->input);
err:
	return ret;
}

static int switchcon_player_led_brightness_set(struct led_classdev *led,
					       enum led_brightness brightness)
{
	struct device *dev = led->dev->parent;
	struct hid_device *hdev = to_hid_device(dev);
	struct switchcon_ctlr *ctlr;
	int val = 0;
	int i;
	int ret;
	int num;

	ctlr = hid_get_drvdata(hdev);
	if (!ctlr) {
		hid_err(hdev, "No controller data\n");
		return -ENODEV;
	}

	/* determine which player led this is */
	for (num = 0; num < SC_NUM_LEDS; num++) {
		if (&ctlr->leds[num] == led)
			break;
	}
	if (num >= SC_NUM_LEDS)
		return -EINVAL;

	mutex_lock(&ctlr->output_mutex);
	for (i = 0; i < SC_NUM_LEDS; i++) {
		if (i == num)
			val |= brightness << i;
		else
			val |= ctlr->leds[i].brightness << i;
	}
	ret = switchcon_set_player_leds(ctlr, 0, val);
	mutex_unlock(&ctlr->output_mutex);

	return ret;
}

static const char * const switchcon_player_led_names[] = {
	"player1",
	"player2",
	"player3",
	"player4"
};

static DEFINE_MUTEX(switchcon_input_num_mutex);
static int switchcon_player_leds_create(struct switchcon_ctlr *ctlr)
{
	struct hid_device *hdev = ctlr->hdev;
	struct led_classdev *led;
	size_t name_len;
	char *name;
	int ret = 0;
	int i;
	static int input_num = 1;

	/* Set the default controller player leds based on controller number */
	mutex_lock(&switchcon_input_num_mutex);
	mutex_lock(&ctlr->output_mutex);
	ret = switchcon_set_player_leds(ctlr, 0, 0xF >> (4 - input_num));
	if (ret)
		hid_warn(ctlr->hdev, "Failed to set leds; ret=%d\n", ret);
	mutex_unlock(&ctlr->output_mutex);

	/* configure the player LEDs */
	for (i = 0; i < SC_NUM_LEDS; i++) {
		name_len = strlen(switchcon_player_led_names[i])
			   + strlen(dev_name(&hdev->dev)) + 2;
		name = devm_kzalloc(&hdev->dev, name_len, GFP_KERNEL);
		if (!name) {
			ret = -ENOMEM;
			break;
		}
		ret = snprintf(name, name_len, "%s:%s", dev_name(&hdev->dev),
						switchcon_player_led_names[i]);
		if (ret < 0)
			break;

		led = &ctlr->leds[i];
		led->name = name;
		led->brightness = ((i + 1) <= input_num) ? LED_ON : LED_OFF;
		led->max_brightness = LED_ON;
		led->brightness_set_blocking =
					switchcon_player_led_brightness_set;
		led->flags = LED_CORE_SUSPENDRESUME | LED_HW_PLUGGABLE;

		ret = devm_led_classdev_register(&hdev->dev, led);
		if (ret) {
			hid_err(hdev, "Failed registering %s LED\n", led->name);
			break;
		}
	}

	if (++input_num > 4)
		input_num = 1;
	mutex_unlock(&switchcon_input_num_mutex);

	return ret;
}

static int switchcon_battery_get_property(struct power_supply *supply,
					  enum power_supply_property prop,
					  union power_supply_propval *val)
{
	struct switchcon_ctlr *ctlr = power_supply_get_drvdata(supply);
	unsigned long flags;
	int ret = 0;
	u8 capacity;
	bool charging;
	bool powered;

	spin_lock_irqsave(&ctlr->lock, flags);
	capacity = ctlr->battery_capacity;
	charging = ctlr->battery_charging;
	powered = ctlr->host_powered;
	spin_unlock_irqrestore(&ctlr->lock, flags);

	switch(prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = capacity;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (charging)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (capacity == 100 && powered)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static enum power_supply_property switchcon_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_STATUS,
};

static int switchcon_power_supply_create(struct switchcon_ctlr *ctlr)
{
	struct hid_device *hdev = ctlr->hdev;
	struct power_supply_config supply_config = { .drv_data = ctlr, };
	const char * const name_fmt = "nintendo_switch_controller_battery_%s";
	int ret = 0;

	/* Set initially to 100 before receiving first input report */
	ctlr->battery_capacity = 100;

	/* Configure the battery's description */
	ctlr->battery_desc.properties = switchcon_battery_props;
	ctlr->battery_desc.num_properties =
					ARRAY_SIZE(switchcon_battery_props);
	ctlr->battery_desc.get_property = switchcon_battery_get_property;
	ctlr->battery_desc.use_for_apm = 0;
	ctlr->battery_desc.name = devm_kasprintf(&hdev->dev, GFP_KERNEL,
						 name_fmt,
						 dev_name(&hdev->dev));
	if (!ctlr->battery_desc.name)
		return -ENOMEM;

	ctlr->battery = devm_power_supply_register(&hdev->dev,
						   &ctlr->battery_desc,
						   &supply_config);
	if (IS_ERR(ctlr->battery)) {
		ret = PTR_ERR(ctlr->battery);
		hid_err(hdev, "Failed to register battery; ret=%d\n", ret);
		return ret;
	}
	power_supply_powers(ctlr->battery, &hdev->dev);
	return 0;
}

/* data input must have at least 9 bytes */
static void switchcon_parse_lstick_calibration(u8 *data,
					       struct switchcon_ctlr *ctlr)
{
	struct switchcon_stick_cal *cal_x = &ctlr->left_stick_cal_x;
	struct switchcon_stick_cal *cal_y = &ctlr->left_stick_cal_y;
	s32 x_max_above;
	s32 x_min_below;
	s32 y_max_above;
	s32 y_min_below;

	x_max_above = hid_field_extract(ctlr->hdev, (data + 0), 0, 12);
	y_max_above = hid_field_extract(ctlr->hdev, (data + 1), 4, 12);
	cal_x->center = hid_field_extract(ctlr->hdev, (data + 3), 0, 12);
	cal_y->center = hid_field_extract(ctlr->hdev, (data + 4), 4, 12);
	x_min_below = hid_field_extract(ctlr->hdev, (data + 6), 0, 12);
	y_min_below = hid_field_extract(ctlr->hdev, (data + 7), 4, 12);
	cal_x->max = cal_x->center + x_max_above;
	cal_x->min = cal_x->center - x_min_below;
	cal_y->max = cal_y->center + y_max_above;
	cal_y->min = cal_y->center - y_min_below;
}

/* data input must have at least 9 bytes */
static void switchcon_parse_rstick_calibration(u8 *data,
					       struct switchcon_ctlr *ctlr)
{
	struct switchcon_stick_cal *cal_x = &ctlr->right_stick_cal_x;
	struct switchcon_stick_cal *cal_y = &ctlr->right_stick_cal_y;
	s32 x_max_above;
	s32 x_min_below;
	s32 y_max_above;
	s32 y_min_below;

	cal_x->center = hid_field_extract(ctlr->hdev, (data + 0), 0, 12);
	cal_y->center = hid_field_extract(ctlr->hdev, (data + 1), 4, 12);
	x_min_below = hid_field_extract(ctlr->hdev, (data + 3), 0, 12);
	y_min_below = hid_field_extract(ctlr->hdev, (data + 4), 4, 12);
	x_max_above = hid_field_extract(ctlr->hdev, (data + 6), 0, 12);
	y_max_above = hid_field_extract(ctlr->hdev, (data + 7), 4, 12);
	cal_x->max = cal_x->center + x_max_above;
	cal_x->min = cal_x->center - x_min_below;
	cal_y->max = cal_y->center + y_max_above;
	cal_y->min = cal_y->center - y_min_below;
}

/* Common handler for parsing inputs */
static int switchcon_ctlr_read_handler(struct switchcon_ctlr *ctlr,
						u8 *data, int size)
{
	int ret = 0;

	switch (data[0]) {
	case SC_INPUT_SUBCMD_REPLY:
	case SC_INPUT_IMU_DATA:
	case SC_INPUT_MCU_DATA:
		if (size >= 12) /* make sure it contains the input report */
			switchcon_parse_report(ctlr, data);
		break;
	default:
		break;
	}

	return ret;
}

static int switchcon_ctlr_handle_event(struct switchcon_ctlr *ctlr, u8 *data,
								    int size)
{
	int ret = 0;
	bool match = false;
	int copy_size;
	struct switchcon_input_report *report;

	if (unlikely(mutex_is_locked(&ctlr->output_mutex)) &&
	    ctlr->msg_type != SWITCHCON_MSG_TYPE_NONE) {
		switch (ctlr->msg_type) {
		case SWITCHCON_MSG_TYPE_USB:
			if (size < 2)
				break;
			if (data[0] == SC_INPUT_USB_RESPONSE &&
			    data[1] == ctlr->usb_ack_match)
				match = true;
			break;
		case SWITCHCON_MSG_TYPE_SUBCMD:
			if (size < sizeof(struct switchcon_input_report) ||
			    data[0] != SC_INPUT_SUBCMD_REPLY)
				break;
			report = (struct switchcon_input_report *)data;
			if (report->reply.id == ctlr->subcmd_ack_match)
				match = true;
			break;
		default:
			break;
		}

		if (match) {
			if (size > SC_MAX_RESP_SIZE)
				copy_size = SC_MAX_RESP_SIZE;
			else
				copy_size = size;
			memcpy(ctlr->input_buf, data, copy_size);
			ctlr->msg_type = SWITCHCON_MSG_TYPE_NONE;
			ctlr->received_resp = true;
			wake_up(&ctlr->wait);

			/* This message has been handled */
			return 1;
		}
	}

	if (ctlr->ctlr_state == SWITCHCON_CTLR_STATE_READ)
		ret = switchcon_ctlr_read_handler(ctlr, data, size);

	return ret;
}

static int switchcon_hid_event(struct hid_device *hdev,
			struct hid_report *report, u8 *raw_data, int size)
{
	struct switchcon_ctlr *ctlr = hid_get_drvdata(hdev);

	if (size < 1)
		return -EINVAL;

	return switchcon_ctlr_handle_event(ctlr, raw_data, size);
}

static struct switchcon_ctlr *switchcon_ctlr_create(struct hid_device *hdev)
{
	struct switchcon_ctlr *ctlr;

	ctlr = devm_kzalloc(&hdev->dev, sizeof(*ctlr), GFP_KERNEL);
	if (!ctlr)
		return ERR_PTR(-ENOMEM);

	switch (hdev->product) {
	case USB_DEVICE_ID_NINTENDO_PROCON:
		ctlr->type = SWITCHCON_CTLR_TYPE_PROCON;
		break;
	case USB_DEVICE_ID_NINTENDO_JOYCONL:
		ctlr->type = SWITCHCON_CTLR_TYPE_JOYCON_L;
		break;
	case USB_DEVICE_ID_NINTENDO_JOYCONR:
		ctlr->type = SWITCHCON_CTLR_TYPE_JOYCON_R;
		break;
	default:
		return ERR_PTR(-EINVAL);
	}
	ctlr->hdev = hdev;
	ctlr->ctlr_state = SWITCHCON_CTLR_STATE_INIT;
	hid_set_drvdata(hdev, ctlr);
	mutex_init(&ctlr->output_mutex);
	init_waitqueue_head(&ctlr->wait);
	spin_lock_init(&ctlr->lock);
	return ctlr;
}

static void switchcon_ctlr_destroy(struct switchcon_ctlr *ctlr)
{
	if (ctlr->input)
		input_unregister_device(ctlr->input);
	mutex_destroy(&ctlr->output_mutex);
}

static int switchcon_hid_probe(struct hid_device *hdev,
			       const struct hid_device_id *id)
{
	int ret;
	struct switchcon_ctlr *ctlr;
	struct switchcon_input_report *report;
	u8 *raw_cal;

	hid_dbg(hdev, "probe - start\n");

	ctlr = switchcon_ctlr_create(hdev);
	if (IS_ERR(ctlr)) {
		hid_err(hdev, "Failed to create new controller\n");
		ret = PTR_ERR(ctlr);
		goto err;
	}

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "HID parse failed\n");
		goto err;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret) {
		hid_err(hdev, "HW start failed\n");
		goto err;
	}

	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "cannot start hardware I/O\n");
		goto err_stop;
	}

	hid_device_io_start(hdev);

	/* Initialize the controller */
	mutex_lock(&ctlr->output_mutex);
	/* if baudrate command fails, assume ble pro controller */
	if (ctlr->type == SWITCHCON_CTLR_TYPE_PROCON &&
	    !switchcon_send_usb(ctlr, SC_USB_CMD_BAUDRATE_3M)) {
		/* handshake */
		ret = switchcon_send_usb(ctlr, SC_USB_CMD_HANDSHAKE);
		if (ret) {
			hid_err(hdev, "Failed handshake; ret=%d\n", ret);
			goto err_mutex;
		}
		/*
		 * Set no timeout (to keep controller in USB mode).
		 * This doesn't send a response, so ignore the timeout.
		 */
		switchcon_send_usb(ctlr, SC_USB_CMD_NO_TIMEOUT);
	}

	/* get controller calibration data, and parse it */
	ret = switchcon_request_calibration(ctlr);
	if (ret) {
		hid_err(hdev, "Failed to retrieve calibration; ret=%d\n", ret);
		goto err_mutex;
	}
	report = (struct switchcon_input_report *)ctlr->input_buf;
	raw_cal = SC_SUBCMD_REPLY_GET_DATA(&report->reply) + 5;
	switchcon_parse_lstick_calibration(raw_cal, ctlr);
	switchcon_parse_rstick_calibration(raw_cal + 9, ctlr);
	hid_info(ctlr->hdev, "calibration:\n"
			     "l_x_c=%d l_x_max=%d l_x_min=%d\n"
			     "l_y_c=%d l_y_max=%d l_y_min=%d\n"
			     "r_x_c=%d r_x_max=%d r_x_min=%d\n"
			     "r_y_c=%d r_y_max=%d r_y_min=%d\n",
			     ctlr->left_stick_cal_x.center,
			     ctlr->left_stick_cal_x.max,
			     ctlr->left_stick_cal_x.min,
			     ctlr->left_stick_cal_y.center,
			     ctlr->left_stick_cal_y.max,
			     ctlr->left_stick_cal_y.min,
			     ctlr->right_stick_cal_x.center,
			     ctlr->right_stick_cal_x.max,
			     ctlr->right_stick_cal_x.min,
			     ctlr->right_stick_cal_y.center,
			     ctlr->right_stick_cal_y.max,
			     ctlr->right_stick_cal_y.min);

	/* Set the reporting mode to 0x30, which is the full report mode */
	ret = switchcon_set_report_mode(ctlr);
	if (ret) {
		hid_err(hdev, "Failed to set report mode; ret=%d\n", ret);
		goto err_mutex;
	}

	mutex_unlock(&ctlr->output_mutex);

	ret = switchcon_input_create(ctlr);
	if (ret) {
		hid_err(hdev, "Failed to create input device; ret=%d\n", ret);
		goto err_close;
	}

	/* Initialize the leds */
	ret = switchcon_player_leds_create(ctlr);
	if (ret) {
		hid_err(hdev, "Failed to create leds; ret=%d\n", ret);
		goto err_close;
	}

	/* Initialize the battery power supply */
	ret = switchcon_power_supply_create(ctlr);
	if (ret) {
		hid_err(hdev, "Failed to create power_supply; ret=%d\n", ret);
		goto err_close;
	}

	ctlr->ctlr_state = SWITCHCON_CTLR_STATE_READ;

	hid_dbg(hdev, "probe - success\n");
	return 0;

err_mutex:
	mutex_unlock(&ctlr->output_mutex);
err_close:
	switchcon_ctlr_destroy(ctlr);
	hid_hw_close(hdev);
err_stop:
	hid_hw_stop(hdev);
err:
	hid_err(hdev, "probe - fail = %d\n", ret);
	return ret;
}

static void switchcon_hid_remove(struct hid_device *hdev)
{
	struct switchcon_ctlr *ctlr = hid_get_drvdata(hdev);

	hid_dbg(hdev, "remove\n");
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
	switchcon_ctlr_destroy(ctlr);
}

static const struct hid_device_id switchcon_hid_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_PROCON) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_PROCON) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_JOYCONL) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_JOYCONR) },
	{ }
};
MODULE_DEVICE_TABLE(hid, switchcon_hid_devices);

static struct hid_driver switchcon_hid_driver = {
	.name		= "switchcon",
	.id_table	= switchcon_hid_devices,
	.probe		= switchcon_hid_probe,
	.remove		= switchcon_hid_remove,
	.raw_event	= switchcon_hid_event,
};
module_hid_driver(switchcon_hid_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel J. Ogorchock <djogorchock@gmail.com>");
MODULE_DESCRIPTION("Driver for Nintendo Switch Controllers");
