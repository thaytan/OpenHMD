/*
 * Pose estimation using OpenCV
 * Copyright 2015 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+ or BSL-1.0
 */
#ifndef __OPENCV_H__
#define __OPENCV_H__

#include "rift.h"
#include "rift-sensor-device.h"
#include "rift-sensor-maths.h"
#include "rift-sensor-blobwatch.h"

#if HAVE_OPENCV
bool estimate_initial_pose(struct blob *blobs, int num_blobs,
			   int device_id, rift_led *leds, int num_leds,
         rift_sensor_camera_params *calib,
			   posef *pose, int *num_leds_out,
			   int *num_inliers, bool use_extrinsic_guess);

void undistort_points (struct blob *blobs, int num_blobs,
				vec3f *out_points,
        rift_sensor_camera_params *calib);

void rift_project_points(rift_led *leds, int num_led_pos,
  rift_sensor_camera_params *calib, posef *pose, vec3f *out_points);

bool refine_pose(struct blob *blobs, int num_blobs,
		int device_id, rift_led *leds, int num_leds,
		rift_sensor_camera_params *calib,
		posef *pose);
#else
static inline
bool estimate_initial_pose(struct blob *blobs, int num_blobs,
			   int device_id, rift_led *leds, int num_leds,
         rift_sensor_camera_params *calib,
			   posef *pose, int *num_leds_out,
			   int *num_inliers, bool use_extrinsic_guess)
{
	(void)blobs;
	(void)num_blobs;
	(void)device_id;
	(void)leds;
	(void)num_leds;
  (void)calib;
	(void)pose;
	(void)num_leds_out;
	(void)num_inliers;
	(void)use_extrinsic_guess;
  return false;
}
#define rift_project_points(leds,num_led_pos,calib,pose,out_points)

#define undistort_points(blobs,num_blobs,out_points,calib)

#define refine_pose(blobs,num_blobs,device_id,leds,num_leds,calib,pose)

#endif /* HAVE_OPENCV */

#endif /* __OPENCV_H__ */
