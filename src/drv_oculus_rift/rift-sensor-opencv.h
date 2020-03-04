/*
 * Pose estimation using OpenCV
 * Copyright 2015 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+ or BSL-1.0
 */
#ifndef __OPENCV_H__
#define __OPENCV_H__

#include "rift.h"
#include "rift-sensor-maths.h"
#include "rift-sensor-blobwatch.h"

#if HAVE_OPENCV
bool estimate_initial_pose(struct blob *blobs, int num_blobs,
			   rift_led *leds, int num_leds,
			   dmat3 *camera_matrix, double dist_coeffs[5], bool dist_fisheye,
			   quatf *rot, vec3f *trans, int *num_leds_out,
			   int *num_inliers, bool use_extrinsic_guess);

void undistort_points (struct blob *blobs, int num_blobs,
				vec3f *out_points,
				double camera_matrix[9], double dist_coeffs[4], bool dist_fisheye);

void rift_project_points(rift_led *leds, int num_led_pos,
  dmat3 *camera_matrix, double dist_coeffs[5], bool dist_fisheye,
  quatf *rot, vec3f *trans,
  vec3f *out_points);

void undistort_points (struct blob *blobs, int num_blobs,
        vec3f *out_points,
        double camera_matrix[9], double dist_coeffs[5], bool dist_fisheye);

void refine_pose(vec3f *image_points,
    rift_led **leds, int num_matches,
    quatf *rot, vec3f *trans, double *reprojection_error);
#else
static inline
bool estimate_initial_pose(struct blob *blobs, int num_blobs,
			   rift_led *leds, int num_leds,
			   dmat3 *camera_matrix, double dist_coeffs[5], bool dist_fisheye,
			   quatf *rot, vec3f *trans, int *num_leds_out,
			   int *num_inliers, bool use_extrinsic_guess);
{
	(void)blobs;
	(void)num_blobs;
	(void)leds;
	(void)num_leds;
	(void)camera_matrix;
	(void)dist_coeffs;
	(void)dist_fisheye;
	(void)rot;
	(void)trans;
	(void)num_leds_out;
	(void)num_inliers;
	(void)use_extrinsic_guess;
  return false;
}
#define rift_project_points(leds,num_led_pos,camera_matrix,dist_coeffs,dist_fisheye,rot,trans,out_points)

#define undistort_points (blobs,num_blobs,out_points,camera_matrix,dist_coeffs,dist_fisheye)

#define refine_pose(image_points,leds,num_matches,rot,trans,reprojection_error)

#endif /* HAVE_OPENCV */

#endif /* __OPENCV_H__ */
