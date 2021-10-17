// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift Driver Internal Interface */
#ifndef __RIFT_DEBUG_DRAW_H
#define __RIFT_DEBUG_DRAW_H

#include "rift-tracker-common.h"
#include "rift-sensor.h"
#include "rift-sensor-blobwatch.h"
#include "correspondence_search.h"

#if HAVE_PIPEWIRE
void rift_debug_draw_frame (uint8_t *pixels, struct blobservation* bwobs,
	correspondence_search_t *cs, struct rift_sensor_analysis_frame *frame,
	uint8_t n_devs, rift_tracked_device **devs,
	rift_sensor_camera_params *calib, posef *camera_pose);
#else
#define rift_debug_draw_frame(pixels,bwobs,cs,frame,n_devs,devs,calib,camera_pose)
#endif

#endif
