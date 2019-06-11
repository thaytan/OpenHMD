/*
 * Pose estimation using OpenCV
 * Copyright 2015 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+ or BSL-1.0
 */
#ifndef __OPENCV_H__
#define __OPENCV_H__

#include "rift.h"
#include "rift-sensor-maths.h"

#if HAVE_OPENCV
bool estimate_initial_pose(struct blob *blobs, int num_blobs,
			   rift_led *leds, int num_leds,
			   dmat3 *camera_matrix, double dist_coeffs[5],
			   dquat *rot, dvec3 *trans, bool use_extrinsic_guess);
#else
static inline
bool estimate_initial_pose(struct blob *blobs, int num_blobs,
			   rift_led *leds, int num_leds,
			   dmat3 *camera_matrix, double dist_coeffs[5],
			   dquat *rot, dvec3 *trans, bool use_extrinsic_guess)
{
	(void)blobs;
	(void)num_blobs;
	(void)leds;
	(void)num_leds;
	(void)camera_matrix;
	(void)dist_coeffs;
	(void)rot;
	(void)trans;
	(void)use_extrinsic_guess;
  return false;
}
#endif /* HAVE_OPENCV */

#endif /* __OPENCV_H__ */
