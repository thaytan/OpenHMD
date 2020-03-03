// Copyright 2020 Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
#ifndef __POSE_HELPER_H__
#define __POSE_HELPER_H__

#include "rift.h"
#include "rift-sensor-maths.h"
#include "rift-sensor-blobwatch.h"

#define MAX_OBJECT_LEDS 64

typedef struct {
  int matched_blobs;
  int unmatched_blobs;
  int visible_leds;

  double reprojection_error;

  bool good_pose_match; /* TRUE if rift_evaluate_pose() considered this a good match */
} rift_pose_metrics;

void rift_evaluate_pose (rift_pose_metrics *score, quatf *orient, vec3f *trans,
    struct blob *blobs, int num_blobs,
		rift_led *leds, int num_leds,
		dmat3 *camera_matrix, double dist_coeffs[5], bool dist_fisheye);

#endif
