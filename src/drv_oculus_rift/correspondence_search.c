// Copyright 2020 Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
#define _GNU_SOURCE // For qsort_r FIXME
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>

#include "lambdatwist/lambdatwist_p3p.h"
#include "correspondence_search.h"
#include "rift-sensor-opencv.h"
#include "rift-sensor-blobwatch.h"
#include "rift-sensor-pose-helper.h"

#define DUMP_SCENE 0
#define DUMP_BLOBS  0
#define DUMP_FULL_DEBUG 0
#define DUMP_FULL_LOG 0
#define DUMP_TIMING 0

#define MAX_LED_SEARCH_DEPTH 8

#if DUMP_TIMING
#ifdef DRIVER_OCULUS_RIFT
#include "../openhmdi.h"
#define get_cur_time() ohmd_get_tick()
#else
#define get_cur_time() rift_get_tick()
#endif
#endif

/* This file implements a brute-force correspondence search between LED models and observed IR LED blobs.
 *
 * The goal is to iterate over sets of LED and blob candidates, and test them as correspondences. Groups of 4
 * LEDs and Blobs are collected. The first 3 points are used with the Lambdatwist P3P to calculate a hypothesis
 * pose of the object. If the 4th point matches the hypothesis, it proceeds to a full pose check to evaluate
 * how many other LED/blob points match (inliers), and how closely (reprojection error).
 *
 * To speed up the search, the LED and blobs are iterated using nearest neighbour lists, as blobs that are nearby
 * are much more likely to match LEDs that are near each other.
 *
 * LED models provide a set of 3D LED points and their normals. The led_search_model_t helper takes the set
 * of LED points (rift_leds) and for each one calculates an led_search_candidate list. The list holds a set
 * of neighbour LEDs (LEDs with normals within 90° of the anchor LED) sorted by euclidean distance.
 *
 * Before each search, the caller provides the set of 2D blobs to match against, which are given similar
 * treatment - for each blob the set of neighbours is calculated, sorted by increasing distance.
 *
 * If available the vertical alignment of the device is used as a check on valid poses. Since the IMU can generally
 * extract the gravity vector of each device with some variance, poses that don't match the required gravity direction
 * are rejected, avoiding the full-pose check.
 *
 * Further, if CS_FLAGS_STOP_FOR_STRONG_MATCH is passed and a 'strong' pose match is found, the search is stopped
 * and the result returned immediately.
 * A strong match is defined in the pose-helper, as 7 or more LEDs matching observed blobs within 1.5 pixels/led
 * reprojection error.
 *
 * A full search of all possible LED and blob combinations is prohibitive, so the search is limited to matching
 * neighbours up to MAX_LED_SEARCH_DEPTH deep for LEDs - ie, for MAX_LED_SEARCH_DEPTH=6, we check each LED and
 * combinations of 3 from the 6 nearest LEDs. For blobs, we check MAX_BLOB_SEARCH_DEPTH nearest neighbours, but filter the
 * list to ignore blobs that are already assigned to another device (unless CS_FLAG_MATCH_ALL_BLOBS is set).
 *
 * In general, there are many more possible LEDs than there are visible blobs, and most of the LEDs will not be
 * visible in any given pose. blobs, on the other hand are (by definition) visible - if we find the right LEDs, they'll
 * match. As such, it's better to loop over some 1-2 depth combinations of blobs and test them against nearest LEDs,
 * before proceeding to any deeper search depths. If CS_FLAG_SHALLOW_SEARCH is provided, only depth 1/2 is checked.
 *
 */

//#define printf(s,...)
#define abs(x) ((x) >= 0 ? (x) : -(x))

correspondence_search_t *
correspondence_search_new(rift_sensor_camera_params *camera_calib)
{
    correspondence_search_t *cs = calloc(1, sizeof (correspondence_search_t));
    cs->calib = camera_calib;
    return cs;
}

#if DUMP_FULL_DEBUG
#define DEBUG(s,...) printf(s,__VA_ARGS__)
#else
#define DEBUG(s,...)
#endif

#if DUMP_TIMING
#define DEBUG_TIMING(s,...) printf(s,__VA_ARGS__)
#else
#define DEBUG_TIMING(s,...)
#endif

#undef LOG
#if DUMP_FULL_LOG
#define LOG(s,...) printf(s,__VA_ARGS__)
#else
#define LOG(s,...)
#endif

#define MIN(a,b) ((a) < (b) ? a : b)
#define MAX(a,b) ((a) > (b) ? a : b)

static int compare_blobs_distance (const void *elem1, const void *elem2, void *arg);

void
correspondence_search_set_blobs (correspondence_search_t *cs, struct blob *blobs, int num_blobs)
{
    int i, m;
    vec3f undistorted_points[MAX_BLOBS_PER_FRAME];
    cs_image_point_t *blob_list[MAX_BLOBS_PER_FRAME];

    assert (num_blobs <= MAX_BLOBS_PER_FRAME);

    /* Updating the blobs means we don't know if we have good poses any more */
    for (m = 0; m < cs->num_models; m++) {
        cs_model_info_t *mi = &cs->models[m];
        mi->match_flags = 0;
    }

    if (cs->points != NULL)
        free (cs->points);

    cs->points = calloc(num_blobs, sizeof (cs_image_point_t));
    cs->num_points = num_blobs;
    cs->blobs = blobs;

    /* Undistort points so we can project / match properly */
    undistort_points (blobs, num_blobs, undistorted_points, cs->calib);

#if DUMP_BLOBS
    printf ("Building blobs search array\n");
#endif
    for (i = 0; i < num_blobs; i++) {
        cs_image_point_t *p = cs->points + i;
        struct blob *b = blobs + i;

        p->point_homog[0] = undistorted_points[i].x;
        p->point_homog[1] = undistorted_points[i].y;
        p->point_homog[2] = 1;

        p->size[0] = b->width / cs->calib->camera_matrix.m[0];
        p->size[1] = b->height / cs->calib->camera_matrix.m[4];
        p->max_size = MAX (p->size[0], p->size[1]);

        p->blob = b;
        blob_list[i] = p;

#if DUMP_BLOBS
        printf ("Blob %u = (%f,%f %ux%u) -> (%f, %f) %f x %f (LED id %d)\n", i,
                b->x, b->y, b->width, b->height,
                p->point_homog[0], p->point_homog[1],
                p->size[0], p->size[1], b->led_id);
#endif
    }

    /* Now the blob_list is populated,
     * loop over the blob list and for each, prepare a list of neighbours sorted by distance */
    for (i = 0; i < cs->num_points; i++) {
        cs_image_point_t *anchor = cs->points + i;

        /* Sort the blobs by proximity to anchor blob */
        qsort_r (blob_list, cs->num_points, sizeof (cs_image_point_t *), compare_blobs_distance, anchor);
        memcpy (cs->blob_neighbours[i], blob_list, cs->num_points * sizeof(cs_image_point_t *));
    }
}

bool
correspondence_search_set_model (correspondence_search_t *cs, int id, led_search_model_t *model)
{
    if (cs->num_models == CS_MAX_MODELS)
        return false;

    cs->models[cs->num_models].id = id;
    cs->models[cs->num_models].model = model;
    cs->num_models++;

    return true;
}

void
correspondence_search_free (correspondence_search_t *cs)
{
    if (cs->points)
      free (cs->points);
    free (cs);
}

static int compare_blobs_distance (const void *elem1, const void *elem2, void *arg)
{
  const cs_image_point_t *b1 = * (const cs_image_point_t **) elem1;
  const cs_image_point_t *b2 = * (const cs_image_point_t **) elem2;
  cs_image_point_t *anchor = arg;
  double dist1, dist2;

  dist1 = (b1->blob->y - anchor->blob->y) * (b1->blob->y - anchor->blob->y) +
          (b1->blob->x - anchor->blob->x) * (b1->blob->x - anchor->blob->x);
  dist2 = (b2->blob->y - anchor->blob->y) * (b2->blob->y - anchor->blob->y) +
          (b2->blob->x - anchor->blob->x) * (b2->blob->x - anchor->blob->x);

  if (dist1 > dist2) return  1;
  if (dist1 < dist2) return -1;

  return 0;
}

#if DUMP_SCENE
static void
dump_pose (correspondence_search_t *cs, led_search_model_t *model, posef *pose, cs_model_info_t *mi)
{
    int i;
	  rift_leds *leds = model->leds;

    printf ("pose_points[%i] = [\n", mi->id);
    for (i = 0; i < leds->num_points; i++) {
      vec3f pos, dir;

      /* Project HMD LED into the image (no distortion) */
      oquatf_get_rotated(&pose->orient, &leds->points[i].pos, &pos);
      ovec3f_add (&pos, &pose->pos, &pos);
      if (pos.z < 0)
        continue; // Can't be behind the camera

      ovec3f_multiply_scalar (&pos, 1.0/pos.z, &pos);

      double x = pos.x;
      double y = pos.y;

      ovec3f_normalize_me(&pos);

      oquatf_get_rotated(&pose->orient, &leds->points[i].dir, &dir);
      ovec3f_normalize_me(&dir);

      double facing_dot;
      facing_dot = ovec3f_get_dot (&pos, &dir);

      if (facing_dot < 0) {
          printf ("  (%f,%f),\n", x, y);
      }
    }
    printf ("]\n");
}
#endif

static bool
correspondence_search_project_pose (correspondence_search_t *cs, led_search_model_t *model,
	posef *pose, cs_model_info_t *mi, int depth)
{
  /* Given a pose, project each 3D LED point back to 2D and assign correspondences to blobs */
  /* If enough match, print out the pose */
	rift_leds *leds = model->leds;

  /* Increment stats */
  cs->num_pose_checks++;

  if (pose->pos.z < 0.05 || pose->pos.z > 15) { /* Invalid position, out of range */
		LOG("Pose out of range - Z @ %f metres\n", pose->pos.z);
    return false;
  }

	/* See if we need to make a gravity vector alignment check */
	if (mi->search_flags & CS_FLAG_MATCH_GRAVITY) {
		quatf pose_gravity_swing, pose_gravity_twist;

		oquatf_decompose_swing_twist(&pose->orient, &mi->gravity_vector, &pose_gravity_swing, &pose_gravity_twist);

		float pose_angle = acosf(oquatf_get_dot(&pose_gravity_swing, &mi->gravity_swing));
		if (pose_angle > mi->gravity_tolerance_rad) {
			DEBUG("model %d failed pose match - orientation was not within tolerance (error %f deg > %f deg)\n"
			    "gravity vec %f %f %f pose %f %f %f %f swing %f %f %f %f prior swing %f %f %f %f\n",
			    mi->id, RAD_TO_DEG(pose_angle), RAD_TO_DEG(mi->gravity_tolerance_rad),
			    mi->gravity_vector.x, mi->gravity_vector.y, mi->gravity_vector.z,
			    pose->orient.x, pose->orient.y, pose->orient.z, pose->orient.w,
			    pose_gravity_swing.x, pose_gravity_swing.y, pose_gravity_swing.z, pose_gravity_swing.w,
			    mi->gravity_swing.x, mi->gravity_swing.y, mi->gravity_swing.z, mi->gravity_swing.w
			    );
			return false;
		}
	}

  rift_pose_metrics score;

  /* Check how many LEDs have matching blobs in this pose,
   * if there's enough we have a good match */
  /* FIXME: It would be better to be able to pass a list of undistorted
   * blob points */
  if (mi->search_flags & CS_FLAG_HAVE_POSE_PRIOR) {
    rift_evaluate_pose_with_prior (&score, pose,
        false, &mi->pose_prior, mi->pos_error_thresh, mi->rot_error_thresh,
        cs->blobs, cs->num_points,
        mi->id, leds->points, leds->num_points, cs->calib, NULL);
  }
  else {
    rift_evaluate_pose (&score, pose, cs->blobs, cs->num_points,
        mi->id, leds->points, leds->num_points, cs->calib, NULL);
  }

  /* If this pose is any good, test it further */
  if (POSE_HAS_FLAGS(&score, RIFT_POSE_MATCH_GOOD)) {
    if (rift_score_is_better_pose (&mi->best_score, &score)) {
        mi->best_score = score;
        mi->best_pose = *pose;
#if DUMP_TIMING
        mi->best_pose_found_time = get_cur_time();
#endif
        mi->best_pose_blob_depth = depth;
        mi->best_pose_led_depth = mi->led_depth;
        mi->match_flags = mi->best_score.match_flags;

        if (mi->match_flags & RIFT_POSE_MATCH_STRONG) {
          DEBUG_TIMING("# Found a strong match for model %d - %d points out of %d with error %f pixels^2 after %u trials and %u pose checks\n", mi->id, mi->best_score.matched_blobs,
                    mi->best_score.visible_leds, mi->best_score.reprojection_error, cs->num_trials, cs->num_pose_checks);
          DEBUG_TIMING("# Found at LED depth %d blob depth %d after %f ms. Anchor indices: LED %d blob %d\n",
                    mi->best_pose_led_depth, mi->best_pose_blob_depth, (mi->best_pose_found_time - mi->search_start_time) * 1000.0,
                    mi->led_index, mi->blob_index);
          DEBUG_TIMING("# pose orient %f %f %f %f pos %f %f %f\n",
                    mi->best_pose.orient.x, mi->best_pose.orient.y, mi->best_pose.orient.z, mi->best_pose.orient.w,
                    mi->best_pose.pos.x, mi->best_pose.pos.y, mi->best_pose.pos.z);
        }

#if DUMP_FULL_DEBUG
        float error_per_led = score.visible_leds > 0 ? score.reprojection_error / score.visible_leds : -1;
        DEBUG("model %d new best pose candidate orient %f %f %f %f pos %f %f %f has %u visible LEDs, error %f (%f / LED) after %u trials and %u pose checks\n",
               mi->id, pose->orient.x, pose->orient.y, pose->orient.z, pose->orient.w,
                    pose->pos.x, pose->pos.y, pose->pos.z, score.visible_leds, score.reprojection_error, error_per_led,
                    cs->num_trials, cs->num_pose_checks);

        DEBUG("model %d matched %u blobs of %u\n", mi->id, score.matched_blobs, score.visible_leds);
#endif
#if DUMP_SCENE
        dump_pose (cs, model, pose, mi);
#endif
        return true;
    } else {
#if DUMP_FULL_DEBUG
      float error_per_led = score.visible_leds > 0 ? score.reprojection_error / score.visible_leds : -1;
      DEBUG("pose candidate orient %f %f %f %f pos %f %f %f has %u visible LEDs, error %f (%f / LED)\n",
              pose->orient.x, pose->orient.y, pose->orient.z, pose->orient.w,
              pose->pos.x, pose->pos.y, pose->pos.z, score.visible_leds, score.reprojection_error,
              error_per_led);
      DEBUG("matched %u blobs of %u\n", score.matched_blobs, score.visible_leds);
#endif
    }
    LOG("Found good pose match for device %u - %u LEDs matched %u visible ones\n",
        mi->id, score.matched_blobs, score.visible_leds);
    return true;
  }

  LOG("Failed pose match - only %u blobs matched %u visible LEDs. %u unmatched blobs. Error %f\n",
      score.matched_blobs, score.visible_leds, score.unmatched_blobs, score.reprojection_error);

  return false;
}

static void
oquatf_from_rotation_matrix (quatf* me, double R[9])
{
  float trace = R[0] + R[3 + 1] + R[6+2];
  if (trace > 0) {
    float s = 0.5f / sqrtf(trace+ 1.0f);
    me->w = -0.25f / s;
    me->x = -( R[6+1] - R[3+2] ) * s;
    me->y = -( R[2] - R[6] ) * s;
    me->z = -( R[3] - R[1] ) * s;
  } else {
    if ( R[0] > R[4] && R[0] > R[8] ) {
      float s = 2.0f * sqrtf( 1.0f + R[0] - R[3+1] - R[6+2]);
      me->w = -(R[6+1] - R[3+2] ) / s;
      me->x = -0.25f * s;
      me->y = -(R[1] + R[3] ) / s;
      me->z = -(R[2] + R[6] ) / s;
    } else if (R[3+1] > R[6+2]) {
      float s = 2.0f * sqrtf( 1.0f + R[3+1] - R[0] - R[6+2]);
      me->w = -(R[2] - R[6] ) / s;
      me->x = -(R[1] + R[3] ) / s;
      me->y = -0.25f * s;
      me->z = -(R[3+2] + R[6+1] ) / s;
    } else {
      float s = 2.0f * sqrtf( 1.0f + R[6+2] - R[0] - R[3+1] );
      me->w = -(R[3]   - R[1] ) / s;
      me->x = -(R[2]   + R[6] ) / s;
      me->y = -(R[3+2] + R[6+1] ) / s;
      me->z = -0.25f * s;
    }
  }

}

static void
check_led_against_model_subset (correspondence_search_t *cs, cs_model_info_t *mi,
     cs_image_point_t **blobs, rift_led *model_leds[4], int depth)
{
  led_search_model_t *model = mi->model;
  double x[3][3];
  vec3f *xcheck;
  int i;
  double *y1, *y2, *y3;
  double Rs[4][9], Ts[4][3];
  vec3f checkblob, blob0;

  y1 = blobs[0]->point_homog;
  y2 = blobs[1]->point_homog;
  y3 = blobs[2]->point_homog;

  blob0.x = blobs[0]->point_homog[0];
  blob0.y = blobs[0]->point_homog[1];
  blob0.z = blobs[0]->point_homog[2];

  /* 4th point we'll check against */
  checkblob.x = blobs[3]->point_homog[0];
  checkblob.y = blobs[3]->point_homog[1];
  checkblob.z = blobs[3]->point_homog[2];

  cs->num_trials++;

  for (i = 0; i < 3; i++) {
    x[i][0] = model_leds[i]->pos.x;
    x[i][1] = model_leds[i]->pos.y;
    x[i][2] = model_leds[i]->pos.z;
  }
  xcheck = &model_leds[3]->pos;

  /* FIXME: It would be better if this spat out quaternions,
   * then we wouldn't need to convert below */
  int valid = lambdatwist_p3p (y1, y2, y3, x[0], x[1], x[2],
          Rs, Ts);

  if (valid) {
    for (i = 0; i < valid; i++) {
      posef pose;
      vec3f checkpos, checkdir;
      float l;

      /* Construct quat and trans for this pose */
      oquatf_from_rotation_matrix (&pose.orient, Rs[i]);

      pose.pos.x = Ts[i][0];
      pose.pos.y = Ts[i][1];
      pose.pos.z = Ts[i][2];

      if (pose.pos.z < 0.05 || pose.pos.z > 15) {
          /* The object is unlikely to be < 5cm or > 15m from the camera */
          continue;
      }

      /* The quaternion produced should already be normalised, but sometimes it's not! */
#if 1
      oquatf_normalize_me (&pose.orient);
#else
      l = oquatf_get_length (&pose.orient);
      assert (l > 0.9999 && l < 1.0001);
#endif

      /* This pose must yield a projection of the anchor
       * point, or something is really wrong */
      oquatf_get_rotated (&pose.orient, &model_leds[0]->pos, &checkpos);
      ovec3f_add (&checkpos, &pose.pos, &checkpos);

      /* And should be camera facing in this pose */
      oquatf_get_rotated (&pose.orient, &model_leds[0]->dir, &checkdir);
      ovec3f_normalize_me(&checkpos);

      double facing_dot = ovec3f_get_dot (&checkpos, &checkdir);

      /* Require only that the anchor LED not be actively facing away from
       * the camera (that it's at worst perpendicular). Controller LEDs
       * can be visible at that angle */
      if (facing_dot > 0.0) {
        // Anchor LED not facing the camera -> invalid pose
        continue;
      }

      /* Calculate the image plane projection of the anchor
       * LED position and check it's within 2.5mm of where it
       * should be, to catch spurious failures in lambdatwist */
      vec3f tmp;
      ovec3f_multiply_scalar (&checkpos, 1.0/checkpos.z, &checkpos);
      ovec3f_subtract (&checkpos, &blob0, &tmp);
      l = ovec3f_get_length (&tmp);
      if (!(l <= 0.0025)) {
	      printf ("Error pose candidate orient %f %f %f %f pos %f %f %f "
            "Anchor LED %f %f %f projected to %f %f %f (err %f)\n",
            pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w,
            pose.pos.x, pose.pos.y, pose.pos.z,
            blob0.x, blob0.y, blob0.z,
            checkpos.x, checkpos.y, checkpos.z, l);
        continue; /* FIXME: Figure out why this happened */
      }

      /* check against the 4th point to check the proposed P3P solution */
      oquatf_get_rotated (&pose.orient, xcheck, &checkpos);
      ovec3f_add (&checkpos, &pose.pos, &checkpos);
      ovec3f_multiply_scalar (&checkpos, 1.0/checkpos.z, &checkpos);

      /* Subtract projected point from reference point to check error */
      ovec3f_subtract (&checkpos, &checkblob, &tmp);
      float distance = ovec3f_get_length (&tmp);

      /* Check that the 4th point projected to within its blob */
      if (distance <= blobs[3]->max_size) {
#if 0
        /* Convert back to pixels for the debug output */
        ovec3f_multiply_scalar (&checkpos, cs->camera_matrix->m[0], &checkpos);

  	    printf ("model %u pose candidate orient %f %f %f %f pos %f %f %f\n",
            mi->id, pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w,
            pose.pos.x, pose.pos.y, pose.pos.z);
        printf ("4th point @ %f %f offset %f pixels\n",
            checkpos.x, checkpos.y, distance * cs->camera_matrix->m[0]);
#endif
        if (correspondence_search_project_pose (cs, model, &pose, mi, depth)) {
#if 0
          printf ("  P4P points %f,%f,%f -> %f %f\n"
                  "         %f,%f,%f -> %f %f\n"
                  "         %f,%f,%f -> %f %f\n"
                  "         %f,%f,%f -> %f %f\n",
                x[0][0], x[0][1], x[0][2],
                y1[0], y1[1],
                x[1][0], x[1][1], x[1][2],
                y2[0], y2[1],
                x[2][0], x[2][1], x[2][2],
                y3[0], y3[1],
                xcheck->x, xcheck->y, xcheck->z,
                checkblob.x,
                checkblob.y);
#endif
        }
      }
    }
  }
}

/* Select k entries from the n provided in candidate_list into
 * output_list, then call check_led_match() with the result_list */
static void
select_k_blobs_from_n (correspondence_search_t *cs, cs_model_info_t *mi, rift_led **model_leds,
    cs_image_point_t **result_list, cs_image_point_t **output_list,
    cs_image_point_t **candidate_list, int k, int n, int depth)
{
    if (k == 1) {
        output_list[0] = candidate_list[0];
        check_led_against_model_subset (cs, mi, result_list, model_leds, depth);
        return;
    }

    /*
     * 2 branches here:
     *   take the first entry, then select k-1 from n-1
     *   don't take the first entry, select k from n-1 if n > k
     */
    assert (k > 1);
    assert (n > 1);
    output_list[0] = candidate_list[0];
    select_k_blobs_from_n (cs, mi, model_leds, result_list, output_list+1, candidate_list+1, k-1, n-1, depth);

    /* Short circuit if we found a strong pose match already */
    if ((mi->match_flags & RIFT_POSE_MATCH_STRONG) && (mi->search_flags & CS_FLAG_STOP_FOR_STRONG_MATCH))
      return;

    if (n > k)
      select_k_blobs_from_n (cs, mi, model_leds, result_list, output_list, candidate_list+1, k, n-1, depth+1);
}

/* Generate quadruples of neighbouring blobs and pass to check_led_match().
 * We want to
 *   a) preserve the first blob in the list each iteration
 *   b) Select combinations of 3 from the next 'max_search_depth' points.
 *
 * Permutation checking is done in the model LED inner loop, so this inner selection
 * only needs to try combos.
 *
 * We can work in a tmp array so as not to destroy the master list.
 *
 * The caller guarantees that the blob_list has at least max_search_depth+1
 * entries.
 */
static void
check_leds_against_anchor (correspondence_search_t *cs, cs_model_info_t *mi,
    rift_led **model_leds, cs_image_point_t *anchor)
{
  cs_image_point_t *work_list[MAX_BLOB_SEARCH_DEPTH+1];
  int max_blob_search_depth = MIN(anchor->num_neighbours, mi->max_blob_depth);

  if (max_blob_search_depth < 3)
      return; // Not enough blobs to compare against

  work_list[0] = anchor;
  select_k_blobs_from_n (cs, mi, model_leds, work_list, work_list+1, anchor->neighbours, 3, max_blob_search_depth, 1);
}

/* Called with 4 model_leds that need checking against blob combos */
static void
check_led_match (correspondence_search_t *cs, cs_model_info_t *mi, rift_led **model_leds, int depth)
{
    int b;

    mi->led_depth = depth;

    for (b = 0; b < cs->num_points; b++) {
        cs_image_point_t *anchor = cs->points + b;
        mi->blob_index = b;
        check_leds_against_anchor (cs, mi, model_leds, anchor);
    }
}

/* Select k rift_led entries from the n provided in candidate_list into
 * output_list, then call check_led_match() with the result_list */
static void
select_k_leds_from_n (correspondence_search_t *cs,
    cs_model_info_t *mi,
    rift_led **result_list, rift_led **output_list,
    rift_led **candidate_list, int k, int n, int depth)
{
    if (k == 1) {
        rift_led *swap_list[4];

        output_list[0] = candidate_list[0];
        check_led_match (cs, mi, result_list, depth);

        /* Short circuit if we found a strong pose match already */
        if ((mi->match_flags & RIFT_POSE_MATCH_STRONG) && (mi->search_flags & CS_FLAG_STOP_FOR_STRONG_MATCH))
          return;

        /* Check the other orientation of blob 2/3, without
         * affecting result_list that needs to stay intact
         * for other recursive calls */
        swap_list[0] = result_list[0];
        swap_list[1] = result_list[2];
        swap_list[2] = result_list[1];
        swap_list[3] = result_list[3];

        check_led_match (cs, mi, swap_list, depth);

        return;
    }

    /*
     * 2 branches here:
     *   take the first entry, then select k-1 from n-1
     *   don't take the first entry, select k from n-1 if n > k
     */
    assert (k > 1);
    assert (n > 1);
    output_list[0] = candidate_list[0];
    select_k_leds_from_n (cs, mi, result_list, output_list+1, candidate_list+1, k-1, n-1, depth);

    /* Short circuit if we found a strong pose match already */
    if ((mi->match_flags & RIFT_POSE_MATCH_STRONG) && (mi->search_flags & CS_FLAG_STOP_FOR_STRONG_MATCH))
      return;

    if (n > k)
      select_k_leds_from_n (cs, mi, result_list, output_list, candidate_list+1, k, n-1, depth+1);
}

/* Generate quadruples of neighbouring leds and pass to check_led_match().
 * We want to
 *   a) preserve the first led in the list each iteration
 *   b) Select combinations of 3 from the 'max_search_depth' neighbouring LEDs.
 *   c) Try 2 permutations, [0,1,2,3] and [0,2,1,3]. In each case index 0 is
 *   the anchor and index 3 is a disambiguation check on the P3P result
 *
 * We can work in a tmp array so as not to destroy the master list.
 *
 * The caller guarantees that the LED neighbours list has at least max_search_depth
 * entries.
 */
static void
generate_led_match_candidates (correspondence_search_t *cs, cs_model_info_t *mi,
    led_search_candidate_t *c)
{
  rift_led *work_list[MAX_LED_SEARCH_DEPTH+1];

  int max_search_depth = MIN (mi->max_led_depth, c->num_neighbours) - mi->min_led_depth + 1;
  if (max_search_depth < 3)
    return; // Not enough LEDs to compare against

  work_list[0] = c->led;
  select_k_leds_from_n (cs, mi, work_list, work_list+1, c->neighbours + mi->min_led_depth - 1, 3, max_search_depth, mi->min_led_depth);
}

static bool
search_pose_for_model (correspondence_search_t *cs, cs_model_info_t *mi)
{
  led_search_model_t *model = mi->model;
  int b, l;

  /* clear the info for this model */
  memset (&mi->best_score, 0, sizeof (rift_pose_metrics));
  mi->match_flags = 0;

  /* Clear stats */
  cs->num_trials = cs->num_pose_checks = 0;

  /* Configure search params from the flags */
  if (mi->search_flags & CS_FLAG_SHALLOW_SEARCH) {
    mi->max_blob_depth = 4; /* Only test nearest neighbours of blobs +1 */
    mi->max_led_depth = 4;  /* Test LEDs plus 1 neighbour removed */
    mi->min_led_depth = 1;  /* Start with the nearest LED neighbour */
  }
  else {
    mi->max_blob_depth = MAX_BLOB_SEARCH_DEPTH;
    mi->max_led_depth = MAX_LED_SEARCH_DEPTH;
    mi->min_led_depth = 3; /* Start with next nearest LED neighbour that shallow search stopped at */
  }

  if (mi->search_flags & CS_FLAG_DEEP_SEARCH) {
    mi->max_blob_depth = MAX_BLOB_SEARCH_DEPTH;
    mi->max_led_depth = MAX_LED_SEARCH_DEPTH;
  }

#if DUMP_TIMING
  /* Set the search start time */
  mi->search_start_time = get_cur_time();
#endif

  /* filter the list of blobs to unknown, or belonging to this model */
  for (b = 0; b < cs->num_points; b++) {
      cs_image_point_t **all_neighbours = cs->blob_neighbours[b];
      cs_image_point_t *anchor = cs->points + b;
      int out_index = 0, in_index;
			int led_id = anchor->blob->led_id;

      if ((mi->search_flags & CS_FLAG_MATCH_ALL_BLOBS) || led_id == LED_INVALID_ID || LED_OBJECT_ID(led_id) == mi->id) {
				for (in_index = 0; in_index < cs->num_points && out_index < MAX_BLOB_SEARCH_DEPTH; in_index++) {
						cs_image_point_t *p = all_neighbours[in_index];

						/* Don't include the blob in its own neighbours */
						if (anchor->blob == p->blob)
							continue;

						led_id = p->blob->led_id;
						if ((mi->search_flags & CS_FLAG_MATCH_ALL_BLOBS) || led_id == LED_INVALID_ID || LED_OBJECT_ID(led_id) == mi->id)
							anchor->neighbours[out_index++] = p;
				}
			}
      anchor->num_neighbours = out_index;

#if DUMP_FULL_LOG
      printf ("Model %d, blob %d @ %f,%f neighbours %d Search list:\n", mi->id, b, anchor->blob->x, anchor->blob->y, anchor->num_neighbours);
      for (int i = 0; i < anchor->num_neighbours; i++) {
          cs_image_point_t *p1 = anchor->neighbours[i];
          double dist = (p1->blob->y - anchor->blob->y) * (p1->blob->y - anchor->blob->y) +
                        (p1->blob->x - anchor->blob->x) * (p1->blob->x - anchor->blob->x);
          printf ("  LED ID %u (%f,%f) @ %f,%f. Dist %f\n", p1->blob->led_id, p1->point_homog[0], p1->point_homog[1],
              p1->blob->x, p1->blob->y, sqrt(dist));
      }
#endif
  }

  /* Start correspondence search for this model */
  /* At this stage, each image point has a list of the nearest neighbours filtered for this model */
  for (l = 0; l < model->num_points; l++) {
      led_search_candidate_t *c = model->points[l];
      mi->led_index = l;

      generate_led_match_candidates (cs, mi, c);

      if ((mi->match_flags & RIFT_POSE_MATCH_STRONG) && (mi->search_flags & CS_FLAG_STOP_FOR_STRONG_MATCH))
        return true;
  }

  if (mi->match_flags & RIFT_POSE_MATCH_GOOD)
    return true;

  return false;
}

bool
correspondence_search_have_pose (correspondence_search_t *cs, int model_id, posef *pose,
    rift_pose_metrics *score)
{
  int i;

  for (i = 0; i < cs->num_models; i++) {
    cs_model_info_t *mi = &cs->models[i];
    if (mi->id != model_id)
        continue;

    if (mi->match_flags & RIFT_POSE_MATCH_GOOD) {
      *pose = mi->best_pose;
      *score = mi->best_score;
      return true;
    }
  }
  return false;
}

bool correspondence_search_find_one_pose (correspondence_search_t *cs, int model_id, CorrespondenceSearchFlags search_flags, posef *pose,
	vec3f *pos_error_thresh, vec3f *rot_error_thresh,
	vec3f *gravity_vector, float gravity_tolerance_rad, rift_pose_metrics *score)
{
  int i;

  assert (pose != NULL);
  assert (score != NULL);

  /* If neither deep nor shallow search was requested, do a full search */
  if ((search_flags & (CS_FLAG_SHALLOW_SEARCH|CS_FLAG_DEEP_SEARCH)) == 0)
    search_flags |= CS_FLAG_SHALLOW_SEARCH | CS_FLAG_DEEP_SEARCH;

  for (i = 0; i < cs->num_models; i++) {
    cs_model_info_t *mi = &cs->models[i];
    if (mi->id != model_id)
        continue;

    mi->search_flags = search_flags;
    mi->match_flags = 0;

    if (search_flags & CS_FLAG_HAVE_POSE_PRIOR) {
      assert(pos_error_thresh != NULL);
      assert(rot_error_thresh != NULL);

      mi->pose_prior = *pose;
      mi->pos_error_thresh = pos_error_thresh;
      mi->rot_error_thresh = rot_error_thresh;
    }

    if (search_flags & CS_FLAG_MATCH_GRAVITY) {
	    quatf pose_gravity_twist;

      /* We need a pose prior to extract the gravity swing to match */
      assert ((search_flags & CS_FLAG_HAVE_POSE_PRIOR) != 0);
      assert (gravity_vector != NULL);

      mi->gravity_vector = *gravity_vector;
      mi->gravity_tolerance_rad = gravity_tolerance_rad;

      oquatf_decompose_swing_twist(&pose->orient, gravity_vector, &mi->gravity_swing, &pose_gravity_twist);
    }

    if (search_pose_for_model (cs, mi) && (mi->match_flags & RIFT_POSE_MATCH_GOOD)) {
			*pose = mi->best_pose;
      *score = mi->best_score;

      DEBUG_TIMING("# Best %s match for model %d was %d points out of %d with error %f pixels^2\n",
          (search_flags & CS_FLAG_MATCH_GRAVITY) ? "aligned" : "unaligned",
          model_id, mi->best_score.matched_blobs, mi->best_score.visible_leds, mi->best_score.reprojection_error);
      DEBUG_TIMING("# Found at LED depth %d blob depth %d after %f ms of %f ms\n",
                mi->best_pose_led_depth, mi->best_pose_blob_depth, (mi->best_pose_found_time - mi->search_start_time) * 1000.0,
                (get_cur_time() - mi->search_start_time) * 1000.0);
      DEBUG_TIMING("# pose orient %f %f %f %f pos %f %f %f\n",
                mi->best_pose.orient.x, mi->best_pose.orient.y, mi->best_pose.orient.z, mi->best_pose.orient.w,
                mi->best_pose.pos.x, mi->best_pose.pos.y, mi->best_pose.pos.z);
      return true;
    }

    *pose = mi->best_pose;
    *score = mi->best_score;
    return false;
  }

  DEBUG("No model found for id %d!\n", model_id);
  score->match_flags = 0;
  return false;
}
