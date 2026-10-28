// SPDX-License-Identifier: GPL-2.0
/*
 * HID quirks for Xiaomi mice and Focus Pen Pro
 *
 * Copyright (c) 2021 Ilya Skriblovsky
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/hid.h>

#include "hid-ids.h"

/* Fixed Mi Silent Mouse report descriptor */
/* Button's Usage Maximum changed from 3 to 5 to make side buttons work */
#define MI_SILENT_MOUSE_ORIG_RDESC_LENGTH   87
static const __u8 mi_silent_mouse_rdesc_fixed[] = {
	0x05, 0x01,         /*  Usage Page (Desktop),               */
	0x09, 0x02,         /*  Usage (Mouse),                      */
	0xA1, 0x01,         /*  Collection (Application),           */
	0x85, 0x03,         /*      Report ID (3),                  */
	0x09, 0x01,         /*      Usage (Pointer),                */
	0xA1, 0x00,         /*      Collection (Physical),          */
	0x05, 0x09,         /*          Usage Page (Button),        */
	0x19, 0x01,         /*          Usage Minimum (01h),        */
	0x29, 0x05, /* X */ /*          Usage Maximum (05h),        */
	0x15, 0x00,         /*          Logical Minimum (0),        */
	0x25, 0x01,         /*          Logical Maximum (1),        */
	0x75, 0x01,         /*          Report Size (1),            */
	0x95, 0x05,         /*          Report Count (5),           */
	0x81, 0x02,         /*          Input (Variable),           */
	0x75, 0x03,         /*          Report Size (3),            */
	0x95, 0x01,         /*          Report Count (1),           */
	0x81, 0x01,         /*          Input (Constant),           */
	0x05, 0x01,         /*          Usage Page (Desktop),       */
	0x09, 0x30,         /*          Usage (X),                  */
	0x09, 0x31,         /*          Usage (Y),                  */
	0x15, 0x81,         /*          Logical Minimum (-127),     */
	0x25, 0x7F,         /*          Logical Maximum (127),      */
	0x75, 0x08,         /*          Report Size (8),            */
	0x95, 0x02,         /*          Report Count (2),           */
	0x81, 0x06,         /*          Input (Variable, Relative), */
	0x09, 0x38,         /*          Usage (Wheel),              */
	0x15, 0x81,         /*          Logical Minimum (-127),     */
	0x25, 0x7F,         /*          Logical Maximum (127),      */
	0x75, 0x08,         /*          Report Size (8),            */
	0x95, 0x01,         /*          Report Count (1),           */
	0x81, 0x06,         /*          Input (Variable, Relative), */
	0xC0,               /*      End Collection,                 */
	0xC0,               /*  End Collection,                     */
	0x06, 0x01, 0xFF,   /*  Usage Page (FF01h),                 */
	0x09, 0x01,         /*  Usage (01h),                        */
	0xA1, 0x01,         /*  Collection (Application),           */
	0x85, 0x05,         /*      Report ID (5),                  */
	0x09, 0x05,         /*      Usage (05h),                    */
	0x15, 0x00,         /*      Logical Minimum (0),            */
	0x26, 0xFF, 0x00,   /*      Logical Maximum (255),          */
	0x75, 0x08,         /*      Report Size (8),                */
	0x95, 0x04,         /*      Report Count (4),               */
	0xB1, 0x02,         /*      Feature (Variable),             */
	0xC0                /*  End Collection                      */
};

static const __u8 *xiaomi_report_fixup(struct hid_device *hdev, __u8 *rdesc,
				       unsigned int *rsize)
{
	switch (hdev->product) {
	case USB_DEVICE_ID_MI_SILENT_MOUSE:
		if (*rsize == MI_SILENT_MOUSE_ORIG_RDESC_LENGTH) {
			hid_info(hdev, "fixing up Mi Silent Mouse report descriptor\n");
			*rsize = sizeof(mi_silent_mouse_rdesc_fixed);
			return mi_silent_mouse_rdesc_fixed;
		}
		break;
	}
	return rdesc;
}

/* P81C exposes its private 14-byte pressure packet as keyboard Usage 0x57.
 * Keep that packet available to hidraw; it must never become KEY_KPPLUS.
 * Match the parsed layout, rather than replacing a firmware report map.
 * Report 2's real squeeze button is consumed by the tablet-tool owner.
 */
static int xiaomi_input_mapping(struct hid_device *hdev, struct hid_input *hi,
			       struct hid_field *field, struct hid_usage *usage,
			       unsigned long **bit, int *max)
{
	struct hid_report *report = field->report;
	struct hid_field *first, *second;

	if (hdev->bus != BUS_BLUETOOTH ||
	    hdev->vendor != BT_VENDOR_ID_XIAOMI_P81C ||
	    hdev->product != BT_DEVICE_ID_XIAOMI_P81C ||
	    report->type != HID_INPUT_REPORT)
		return 0;

	/* The real squeeze report is forwarded as BTN_STYLUS by the same
	 * pressure/THP tablet owner. Do not also expose F19 as a global key.
	 * Preserve every other usage of this one-byte keyboard array.
	 */
	if (report->id == 2 && report->size == 8 && report->maxfield == 1 &&
	    usage->hid == (HID_UP_KEYBOARD | 0x6e) &&
	    field->report_size == 8 && field->report_count == 1 &&
	    field->logical_minimum == 0x4b && field->logical_maximum == 0x6e &&
	    field->flags == 0)
		return -1;

	if (report->id != 5 ||
	    report->size != 112 || report->maxfield != 2 ||
	    usage->hid != (HID_UP_KEYBOARD | 0x57))
		return 0;

	first = report->field[0];
	second = report->field[1];
	if (first->report_offset != 0 || first->report_size != 16 ||
	    first->report_count != 4 || first->logical_minimum != 0 ||
	    first->logical_maximum != 16384 ||
	    second->report_offset != 64 || second->report_size != 8 ||
	    second->report_count != 6 || second->logical_minimum != 0 ||
	    second->logical_maximum != 255 ||
	    first->flags != (HID_MAIN_ITEM_VARIABLE | HID_MAIN_ITEM_RELATIVE) ||
	    second->flags != (HID_MAIN_ITEM_VARIABLE | HID_MAIN_ITEM_RELATIVE))
		return 0;

	return -1;
}

static int xiaomi_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret;

	/* Keep the independent mouse and keyboard applications as hid-generic
	 * exposed them; combining them would change libinput classification.
	 */
	if (hdev->vendor == BT_VENDOR_ID_XIAOMI_P81C &&
	    hdev->product == BT_DEVICE_ID_XIAOMI_P81C)
		hdev->quirks |= HID_QUIRK_INPUT_PER_APP;

	ret = hid_parse(hdev);
	if (ret)
		return ret;
	return hid_hw_start(hdev, HID_CONNECT_DEFAULT);
}

static const struct hid_device_id xiaomi_devices[] = {
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_XIAOMI, USB_DEVICE_ID_MI_SILENT_MOUSE) },
	{ HID_BLUETOOTH_DEVICE(BT_VENDOR_ID_XIAOMI_P81C, BT_DEVICE_ID_XIAOMI_P81C) },
	{ }
};
MODULE_DEVICE_TABLE(hid, xiaomi_devices);

static struct hid_driver xiaomi_driver = {
	.name = "xiaomi",
	.id_table = xiaomi_devices,
	.probe = xiaomi_probe,
	.report_fixup = xiaomi_report_fixup,
	.input_mapping = xiaomi_input_mapping,
};
module_hid_driver(xiaomi_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ilya Skriblovsky <IlyaSkriblovsky@gmail.com>");
MODULE_DESCRIPTION("HID quirks for Xiaomi mice and Focus Pen Pro");
