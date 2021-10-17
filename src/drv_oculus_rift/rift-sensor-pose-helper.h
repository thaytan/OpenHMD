// Copyright 2020 Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
#ifndef __POSE_HELPER_H__
#define __POSE_HELPER_H__

#include "rift.h"
#include "rift-tracker-common.h"
#include "rift-sensor-device.h"
#include "rift-sensor-maths.h"
#include "rift-sensor-blobwatch.h"

#define MAX_OBJECT_LEDS 64

typedef struct {
 double left;
 double top;
 double right;
 double bottom;
} rift_rect_t;

typedef enum {
	RIFT_POSE_MATCH_GOOD     = 0x1, /* A reasonable pose match - most LEDs matched to within a few pixels error */
	RIFT_POSE_MATCH_STRONG   = 0x2, /* A strong pose match is a match with very low error */
	RIFT_POSE_MATCH_POSITION = 0x4, /* The position of the pose matched the prior well */
	RIFT_POSE_MATCH_ORIENT   = 0x8, /* The orientation of the pose matched the prior well */
	RIFT_POSE_HAD_PRIOR      = 0x10, /* If a pose prior was supplied when calculating the score, then rot/trans_error are set */
} rift_pose_match_flags;

#define POSE_SET_FLAG(score,f) ((score)->match_flags |= (f))
#define POSE_CLEAR_FLAG(score,f) ((score)->match_flags &= ~(f))
#define POSE_HAS_FLAGS(score,f) (((score)->match_flags & (f)) == (f))

typedef struct {
	rift_pose_match_flags match_flags;

	int matched_blobs;
	int unmatched_blobs;
	int visible_leds;

	double reprojection_error;

	vec3f orient_error; /* Rotation error (compared to a prior) */
	vec3f pos_error;		/* Translation error (compared to a prior) */
} rift_pose_metrics;

void rift_evaluate_pose (rift_pose_metrics *score, posef *pose,
	struct blob *blobs, int num_blobs,
	int device_id, rift_led *leds, int num_leds,
	rift_sensor_camera_params *calib,
	rift_rect_t *out_bounds);

void rift_evaluate_pose_with_prior (rift_pose_metrics *score, posef *pose,
	bool prior_must_match, posef *pose_prior, const vec3f *pos_error_thresh, const vec3f *rot_error_thresh,
	struct blob *blobs, int num_blobs,
	int device_id, rift_led *leds, int num_leds,
	rift_sensor_camera_params *calib,
	rift_rect_t *out_bounds);

void rift_mark_matching_blobs (posef *pose,
	struct blob *blobs, int num_blobs,
	int device_id, rift_led *leds, int num_leds,
	rift_sensor_camera_params *calib);

void rift_clear_blob_labels (struct blob *blobs, int num_blobs, int device_id);

bool rift_score_is_better_pose (rift_pose_metrics *old_score, rift_pose_metrics *new_score);

#endif
