// Copyright 2020 Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
#ifndef __CORRESPONDENCE_SEARCH_H__
#define __CORRESPONDENCE_SEARCH_H__

#include "rift.h"
#include "rift-sensor-maths.h"
#include "rift-sensor-blobwatch.h"
#include "rift-sensor-pose-helper.h"
#include "led_search.h"

#define CS_MAX_MODELS 3

typedef struct correspondence_search_s correspondence_search_t;
typedef struct cs_image_point_s cs_image_point_t;
typedef struct cs_model_info_s cs_model_info_t;
typedef enum CorrespondenceSearchFlags CorrespondenceSearchFlags;

#define MAX_BLOB_SEARCH_DEPTH 5

enum CorrespondenceSearchFlags {
	CS_FLAG_NONE = 0x0,
	CS_FLAG_SHALLOW_SEARCH = 0x1,           /* do quick search @ depth 1-2 neighbour depth */
	CS_FLAG_DEEP_SEARCH = 0x2,              /* do deeper searches @ up to MAX_LED_SEARCH_DEPTH/MAX_BLOB_SEARCH_DEPTH */
	CS_FLAG_STOP_FOR_STRONG_MATCH = 0x4,    /* Stop searching if a strong match is found, otherwise search all and return best match */
	CS_FLAG_MATCH_ALL_BLOBS = 0x8,          /* Allow matching against all blobs, not just unlabelled ones or for the current device */
	CS_FLAG_HAVE_POSE_PRIOR = 0x10,         /* If the input obj_cam_pose contains a valid prior */
	CS_FLAG_MATCH_GRAVITY   = 0x20,         /* Use the provided gravity vector to check pose verticality */
};

struct cs_image_point_s {
    struct blob *blob;

    double point_homog[3]; // Homogeneous version of the point
    double size[2];        // w/h of the blob, in homogeneous coordinates
    double max_size;       // Max of W/H

    /* List of the nearest blobs, filtered for the active model */
    int num_neighbours;
    cs_image_point_t *neighbours[MAX_BLOB_SEARCH_DEPTH];
};

struct cs_model_info_s {
    int id;

    led_search_model_t *model;

    double best_pose_found_time; /* Time (in secs) at which the best pose was found */
    int best_pose_blob_depth;    /* Blob neighbor depth the best pose is from */
    int best_pose_led_depth;    /* LED neigbour depth the best pose is from */
		posef best_pose;
    rift_pose_match_flags match_flags;

    rift_pose_metrics best_score;

    /* Search parameters */
    double search_start_time;
    int led_depth;
    int led_index;
    int blob_index;

		CorrespondenceSearchFlags search_flags;
		int min_led_depth, max_led_depth;
		int max_blob_depth;

    /* Valid when CS_FLAG_HAVE_POSE_PRIOR is set */
    posef pose_prior;
    vec3f *pos_error_thresh;
    vec3f *rot_error_thresh;

    /* Used when CS_FLAG_MATCH_GRAVITY is set */
    vec3f gravity_vector;
    quatf gravity_swing;
    float gravity_tolerance_rad;
};

struct correspondence_search_s
{
    int num_points;
    cs_image_point_t *points;
    struct blob *blobs; /* Original blobs structs [num_points] */

    int num_models;
    cs_model_info_t models[CS_MAX_MODELS];

    unsigned int num_trials;
    unsigned int num_pose_checks;

    rift_sensor_camera_params *calib;

    /* List of the nearest blobs for each blob */
    cs_image_point_t *blob_neighbours[MAX_BLOBS_PER_FRAME][MAX_BLOBS_PER_FRAME];
};

correspondence_search_t *correspondence_search_new(rift_sensor_camera_params *camera_calib);
void correspondence_search_set_blobs (correspondence_search_t *cs, struct blob *blobs, int num_blobs);
bool correspondence_search_set_model (correspondence_search_t *cs, int model_id, led_search_model_t *model);
void correspondence_search_free (correspondence_search_t *cs);

bool correspondence_search_find_one_pose (correspondence_search_t *cs, int model_id, CorrespondenceSearchFlags search_flags, posef *pose,
				vec3f *pos_error_thresh, vec3f *rot_error_thresh,
				vec3f *gravity_vector, float gravity_tolerance_rad, rift_pose_metrics *score);
bool correspondence_search_have_pose (correspondence_search_t *cs, int model_id, posef *pose, rift_pose_metrics *score);
#endif
