// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift driver - positional tracking interface */

#include "rift.h"
#include "correspondence_search.h"

#ifndef __RIFT_SENSOR_H__
#define __RIFT_SENSOR_H__

#include "rift-tracker.h"

typedef struct rift_sensor_ctx_s rift_sensor_ctx;

rift_sensor_ctx *rift_sensor_new (ohmd_context* ohmd_ctx, int id, const char *serial_no,
	libusb_context *usb_ctx, libusb_device_handle *usb_devh, rift_tracker_ctx *tracker, const uint8_t radio_id[5]);
void rift_sensor_free (rift_sensor_ctx *sensor_ctx);
bool rift_sensor_set_model (rift_sensor_ctx *sensor_ctx, int model_id, led_search_model_t *model);

#endif
