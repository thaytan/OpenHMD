// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift Driver Internal Interface */
#ifndef __RIFT_DEBUG_DRAW_H
#define __RIFT_DEBUG_DRAW_H

#include "rift-sensor-blobwatch.h"
#include "rift-sensor-tracker.h"
#include "rift-sensor-uvc.h"
#include "correspondence_search.h"

void rift_debug_draw_frame (uint8_t *pixels, struct blobservation* bwobs,
  correspondence_search_t *cs, struct rift_sensor_uvc_stream * stream,
	rift_tracked_device *devs, bool is_cv1,
  dmat3 camera_matrix, bool dist_fisheye, double dist_coeffs[5]);

#endif
