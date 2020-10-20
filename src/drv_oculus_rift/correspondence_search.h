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

#define MAX_BLOB_SEARCH_DEPTH 5

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

		posef best_pose;
    bool good_pose_match;

    rift_pose_metrics best_score;

    /* Search parameters */
    bool match_all_blobs;
    bool match_gravity_vector;
    vec3f gravity_vector;
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

    dmat3 *camera_matrix;
    double *dist_coeffs;
    bool dist_fisheye;

    /* List of the nearest blobs for each blob */
    cs_image_point_t *blob_neighbours[MAX_BLOBS_PER_FRAME][MAX_BLOBS_PER_FRAME];
};

correspondence_search_t *correspondence_search_new(dmat3 *camera_matrix, double *dist_coeffs, bool dist_fisheye);
void correspondence_search_set_blobs (correspondence_search_t *cs, struct blob *blobs, int num_blobs);
bool correspondence_search_set_model (correspondence_search_t *cs, int model_id, led_search_model_t *model);
void correspondence_search_free (correspondence_search_t *cs);

bool correspondence_search_find_one_pose (correspondence_search_t *cs, int model_id, bool match_all_blobs, posef *pose, rift_pose_metrics *score);
bool correspondence_search_find_one_pose_aligned (correspondence_search_t *cs, int model_id, bool match_all_blobs, posef *pose,
				vec3f *gravity_vector, float gravity_tolerance_rad, rift_pose_metrics *score);
bool correspondence_search_have_pose (correspondence_search_t *cs, int model_id, posef *pose, rift_pose_metrics *score);
#endif
