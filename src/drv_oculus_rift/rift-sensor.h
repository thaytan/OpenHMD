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
#include "rift-sensor-uvc.h"

typedef struct rift_sensor_ctx_s rift_sensor_ctx;
typedef struct rift_sensor_capture_frame rift_sensor_capture_frame;

struct rift_sensor_capture_frame {
	rift_sensor_uvc_frame uvc;

	/* Index of the frame in the frames array */
	uint8_t id;

	/* LED blink pattern info - in case we use
	 * LED blinking again in the future? */
	uint64_t led_pattern_sof_ts;
	uint8_t led_pattern_phase;

	/* Device poses at capture time */
	posef capture_world_poses[RIFT_MAX_TRACKED_DEVICES];

	/* Timestamp of complete frame arriving from USB */
	uint64_t frame_delivered_ts;

	/* Timestamp of fast/image analysis thread processing start */
	uint64_t image_analysis_start_ts;

	/* Timestamp of fast/image analysis thread processing finish */
	uint64_t image_analysis_finish_ts;
};

rift_sensor_ctx *rift_sensor_new (ohmd_context* ohmd_ctx, int id, const char *serial_no,
	libusb_context *usb_ctx, libusb_device_handle *usb_devh, rift_tracker_ctx *tracker, const uint8_t radio_id[5]);
void rift_sensor_free (rift_sensor_ctx *sensor_ctx);
bool rift_sensor_set_model (rift_sensor_ctx *sensor_ctx, int model_id, led_search_model_t *model);

#endif
