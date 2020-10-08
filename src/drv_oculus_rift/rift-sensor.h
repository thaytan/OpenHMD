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
typedef struct rift_sensor_device_state rift_sensor_device_state;
typedef struct rift_sensor_capture_frame rift_sensor_capture_frame;

struct rift_sensor_device_state {
	posef capture_world_pose;
	posef final_cam_pose;
	rift_pose_metrics score;
};

struct rift_sensor_capture_frame {
	rift_sensor_uvc_frame uvc;

	/* Index of the frame in the frames array */
	uint8_t id;

	/* LED blink pattern info - in case we use
	 * LED blinking again in the future? */
	uint64_t led_pattern_sof_ts;
	uint8_t led_pattern_phase;
	blobservation* bwobs;

	/* Device poses at capture time */
	rift_sensor_device_state device_state[RIFT_MAX_TRACKED_DEVICES];
	uint8_t n_devices;

	/* Timestamp of complete frame arriving from USB */
	uint64_t frame_delivered_ts;

	/* Timestamp of fast/image analysis thread processing start */
	uint64_t image_analysis_start_ts;

	uint64_t blob_extract_finish_ts;

	/* Timestamp of fast/image analysis thread processing finish */
	uint64_t image_analysis_finish_ts;

	bool need_long_analysis;

	bool long_analysis_found_new_blobs;

	/* Timestamp of long/image analysis thread processing start */
	uint64_t long_analysis_start_ts;

	/* Timestamp of long/image analysis thread processing end */
	uint64_t long_analysis_finish_ts;
};

rift_sensor_ctx *rift_sensor_new (ohmd_context* ohmd_ctx, int id, const char *serial_no,
	libusb_context *usb_ctx, libusb_device_handle *usb_devh, rift_tracker_ctx *tracker, const uint8_t radio_id[5]);
void rift_sensor_free (rift_sensor_ctx *sensor_ctx);
bool rift_sensor_add_device (rift_sensor_ctx *ctx, rift_tracked_device *device);

#endif
