// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift driver - positional tracking interface */

#include "rift.h"
#include "correspondence_search.h"

#ifndef __RIFT_SENSOR_TRACKER_H__
#define __RIFT_SENSOR_TRACKER_H__

typedef struct rift_tracker_ctx_s rift_tracker_ctx;
typedef struct rift_sensor_ctx_s rift_sensor_ctx;
typedef struct rift_tracked_device_s rift_tracked_device;

struct rift_tracked_device_s
{
	int id;
	rift_leds *leds;
	led_search_model_t *led_search;

	fusion *fusion;

	posef pose;
};

rift_tracker_ctx *rift_sensor_tracker_new (ohmd_context* ohmd_ctx,
		const uint8_t radio_id[5]);

void rift_sensor_tracker_add_device (rift_tracker_ctx *ctx, int device_id, fusion *f, rift_leds *leds);
void rift_sensor_tracker_new_exposure (rift_tracker_ctx *ctx, uint8_t led_pattern_phase);
void rift_sensor_tracker_free (rift_tracker_ctx *ctx);

#endif
