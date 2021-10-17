// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift driver - positional tracking interface */
#include <libusb.h>

#include "rift.h"
#include "rift-sensor-device.h"

#ifndef __RIFT_SENSOR_USB_H__
#define __RIFT_SENSOR_USB_H__

#define DK2_PID 0x0201
#define CV1_PID 0x0211

rift_sensor_device *rift_sensor_usb_new (ohmd_context* ohmd_ctx, int id, const char *serial_no,
	libusb_context *usb_ctx, libusb_device_handle *usb_devh, const uint8_t hmd_radio_id[5]);

#endif
