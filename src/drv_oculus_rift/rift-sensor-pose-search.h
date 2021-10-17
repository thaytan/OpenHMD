// Copyright 2020 Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
#ifndef __POSE_SEARCH_H__
#define __POSE_SEARCH_H__

#include "rift.h"
#include "rift-tracker-common.h"
#include "rift-sensor-maths.h"
#include "rift-sensor-blobwatch.h"

#include "correspondence_search.h"

typedef struct rift_pose_finder rift_pose_finder;

typedef bool (*rift_pose_finder_cb) (void *cb_data,
	rift_tracked_device *dev, rift_sensor_analysis_frame *frame,
	posef *obj_world_pose, rift_pose_metrics *score);

struct rift_pose_finder {
	int sensor_id;
	rift_sensor_camera_params *calib;

	/* Updated from fast_analysis_thread */
	bool have_camera_pose;
	posef camera_pose;
	vec3f cam_gravity_vector;

	/* Brute force search */
	correspondence_search_t *cs;

	rift_pose_finder_cb pose_cb;
	void *pose_cb_data;
};

void rift_pose_finder_init(rift_pose_finder *pf, rift_sensor_camera_params *calib,
		rift_pose_finder_cb pose_cb, void *pose_cb_data);
void rift_pose_finder_clear(rift_pose_finder *pf);

void rift_pose_finder_process_blobs_fast(rift_pose_finder *pf, rift_sensor_analysis_frame *frame,
	rift_tracked_device **devs);
void rift_pose_finder_process_blobs_long(rift_pose_finder *pf, rift_sensor_analysis_frame *frame,
	rift_tracked_device **devs);

#endif
