/** @file
 *  @brief HoG Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

enum {
	HIDS_REMOTE_WAKE = BIT(0),
	HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

struct hids_info {
	uint16_t version; /* version number of base USB HID Specification */
	uint8_t code; /* country HID Device hardware is localized for. */
	uint8_t flags;
} __packed;

struct hids_report {
	uint8_t id; /* report id */
	uint8_t type; /* report type */
} __packed;

static struct hids_info info = {
	.version = 0x0000,
	.code = 0x00,
	.flags = HIDS_NORMALLY_CONNECTABLE,
};

enum {
	HIDS_INPUT = 0x01,
	HIDS_OUTPUT = 0x02,
	HIDS_FEATURE = 0x03,
};

static struct hids_report input = {
	.id = 0x01,
	.type = HIDS_INPUT,
};

static uint8_t simulate_input;
static uint8_t ctrl_point;
static uint8_t report_map[] = {
        0x05, 0x01,                    // Usage Page (Generic Desktop)
        0x09, 0x05,                    // Usage (Game Pad)
        0xa1, 0x01,                    // Collection (Application)
        0x85, 0x01,                    //  Report ID (1)

        0x05, 0x01,                    //  Usage Page (Generic Desktop)
        0x75, 0x04,                    //  Report Size (4)
        0x95, 0x01,                    //  Report Count (1)
        0x25, 0x07,                    //  Logical Maximum (7)
        0x46, 0x3b, 0x01,              //  Physical Maximum (315)
        0x65, 0x14,                    //  Unit (Degrees,EngRotation)
        0x09, 0x39,                    //  Usage (Hat switch)
        0x81, 0x42,                    //  Input (Data,Var,Abs,Null)
        0x45, 0x00,                    //  Physical Maximum (0)
        0x65, 0x00,                    //  Unit (None)

        0x75, 0x01,                    //  Report Size (1)
        0x95, 0x04,                    //  Report Count (4)
        0x81, 0x01,                    //  Input (Cnst,Arr,Abs)

        0x05, 0x09,                    //  Usage Page (Button)
        0x15, 0x00,                    //  Logical Minimum (0)
        0x25, 0x01,                    //  Logical Maximum (1)
        0x75, 0x01,                    //  Report Size (1)
        0x95, 0x0d,                    //  Report Count (13)
        0x09, 0x01,                    //  Usage (BTN_SOUTH)
        0x09, 0x02,                    //  Usage (BTN_EAST)
        0x09, 0x04,                    //  Usage (BTN_NORTH)
        0x09, 0x05,                    //  Usage (BTN_WEST)
        0x09, 0x07,                    //  Usage (BTN_TL)
        0x09, 0x08,                    //  Usage (BTN_TR)
        0x09, 0x0b,                    //  Usage (BTN_SELECT)
        0x09, 0x0c,                    //  Usage (BTN_START)
        0x09, 0x0d,                    //  Usage (BTN_MODE)
        0x09, 0x0e,                    //  Usage (BTN_THUMBL)
        0x09, 0x0f,                    //  Usage (BTN_THUMBR)
        0x09, 0x11,                    //  Usage (BTN_TRIGGER_HAPPY1)
        0x09, 0x12,                    //  Usage (BTN_TRIGGER_HAPPY2)
        0x81, 0x02,                    //  Input (Data,Var,Abs)

        0x75, 0x01,                    //  Report Size (1)
        0x95, 0x03,                    //  Report Count (3)
        0x81, 0x01,                    //  Input (Cnst,Arr,Abs)

        0x05, 0x01,                    //  Usage Page (Generic Desktop)
        0x15, 0x01,                    //  Logical Minimum (1)
        0x26, 0xff, 0x00,              //  Logical Maximum (255)
        0x09, 0x01,                    //  Usage (Pointer)
        0xa1, 0x00,                    //  Collection (Physical)
        0x09, 0x30,                    //   Usage (X)
        0x09, 0x31,                    //   Usage (Y)
        0x75, 0x08,                    //   Report Size (8)
        0x95, 0x02,                    //   Report Count (2)
        0x81, 0x02,                    //   Input (Data,Var,Abs)
        0xc0,                          //  End Collection

        0x09, 0x01,                    //  Usage (Pointer)
        0xa1, 0x00,                    //  Collection (Physical)
        0x09, 0x33,                    //   Usage (Rx)
        0x09, 0x34,                    //   Usage (Ry)
        0x75, 0x08,                    //   Report Size (8)
        0x95, 0x02,                    //   Report Count (2)
        0x81, 0x02,                    //   Input (Data,Var,Abs)
        0xc0,                          //  End Collection

	0xc0,                          // End Collection
};

static ssize_t read_info(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr, void *buf,
			  uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
				 sizeof(struct hids_info));
}

static ssize_t read_report_map(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, report_map,
				 sizeof(report_map));
}

static ssize_t read_report(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr, void *buf,
			   uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
				 sizeof(struct hids_report));
}

static void input_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	simulate_input = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

static ssize_t read_input_report(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr, void *buf,
				 uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, NULL, 0);
}

static ssize_t write_ctrl_point(struct bt_conn *conn,
				const struct bt_gatt_attr *attr,
				const void *buf, uint16_t len, uint16_t offset,
				uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(ctrl_point)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

#if CONFIG_SAMPLE_BT_USE_AUTHENTICATION
/* Require encryption using authenticated link-key. */
#define SAMPLE_BT_PERM_READ BT_GATT_PERM_READ_AUTHEN
#define SAMPLE_BT_PERM_WRITE BT_GATT_PERM_WRITE_AUTHEN
#else
/* Require encryption. */
#define SAMPLE_BT_PERM_READ BT_GATT_PERM_READ_ENCRYPT
#define SAMPLE_BT_PERM_WRITE BT_GATT_PERM_WRITE_ENCRYPT
#endif

/* HID Service Declaration */
BT_GATT_SERVICE_DEFINE(hog_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ, read_info, NULL, &info),
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ, read_report_map, NULL, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       SAMPLE_BT_PERM_READ,
			       read_input_report, NULL, NULL),
	BT_GATT_CCC(input_ccc_changed,
		    SAMPLE_BT_PERM_READ | SAMPLE_BT_PERM_WRITE),
	BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ,
			   read_report, NULL, &input),
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT,
			       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE,
			       NULL, write_ctrl_point, &ctrl_point),
);

void hog_init(void)
{
}

#define SW0_NODE DT_ALIAS(sw0)

static struct {
        uint8_t hat;
        uint16_t buttons;
        uint8_t x;
        uint8_t y;
        uint8_t rx;
        uint8_t ry;
} __packed ble_hog_report;

static K_SEM_DEFINE(notification_sem, 1, 1);

#include <zephyr/drivers/led.h>

static const struct device *leds = DEVICE_DT_GET_ONE(gpio_leds);

static void notify_cb(struct bt_conn *conn, void *user_data)
{
	led_off(leds, 0);
	k_sem_give(&notification_sem);
}

void hog_button_loop(void)
{
	const struct gpio_dt_spec sw0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
	int ret;

	gpio_pin_configure_dt(&sw0, GPIO_INPUT);

	for (;;) {
		for (int i = 100; i < 200; i++) {
			k_sem_take(&notification_sem, K_FOREVER);

			struct bt_gatt_notify_params params;
			memset(&params, 0, sizeof(params));
			params.attr = &hog_svc.attrs[5];
			params.data = &ble_hog_report;
			params.len = sizeof(ble_hog_report);
			params.func = notify_cb;

			ble_hog_report.x = i;

			led_on(leds, 0);
			ret = bt_gatt_notify_cb(NULL, &params);
			if (ret) {
				if (ret != -ENOTCONN) {
					printk("notify error %d\n", ret);
				}
				k_sem_give(&notification_sem);
				k_sleep(K_MSEC(1000));
				goto xx;
			}
xx:
			//k_sleep(K_MSEC(2));
		}
	}
}
