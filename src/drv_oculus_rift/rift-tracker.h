// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift driver - positional tracking interface */

#include "rift.h"
#include "correspondence_search.h"

#ifndef __RIFT_TRACKER_H__
#define __RIFT_TRACKER_H__

typedef struct rift_tracker_ctx_s rift_tracker_ctx;
typedef struct rift_tracked_device_s rift_tracked_device;

#define RIFT_MAX_TRACKED_DEVICES 3

struct rift_tracked_device_s
{
	int id;
	rift_leds *leds;
	led_search_model_t *led_search;

	fusion *fusion;
	/* Transform from the fusion pose (which tracks the IMU, oriented to the screens/view)
	 * to the model the camera will see, which is offset and rotated 180 degrees */
	posef fusion_to_model;

	/* The model (HMD/controller) pose -> world transform */
	posef pose;
};

rift_tracker_ctx *rift_tracker_new (ohmd_context* ohmd_ctx,
		const uint8_t radio_id[5]);

void rift_tracker_add_device (rift_tracker_ctx *ctx, int device_id, fusion *f, posef *imu_pose, rift_leds *leds);
void rift_tracker_new_exposure (rift_tracker_ctx *ctx, uint8_t led_pattern_phase);
uint8_t rift_tracker_get_led_pattern_phase (rift_tracker_ctx *ctx, uint64_t *ts);
uint8_t rift_tracker_get_device_list(rift_tracker_ctx *tracker_ctx, rift_tracked_device **dev_list);

void rift_tracker_free (rift_tracker_ctx *ctx);

#endif
