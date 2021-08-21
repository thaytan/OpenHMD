// Copyright 2020 Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include "rift-sensor-pose-helper.h"
#include "rift-sensor-opencv.h"

struct visible_led_info {
	rift_led *led;
	vec3f pos;
	double led_radius;
};

static void expand_rect(rift_rect_t *bounds, double x, double y, double w, double h)
{
	if (x < bounds->left)
		bounds->left = x;
	if (y < bounds->top)
		bounds->top = y;
	if (x + w > bounds->right)
		bounds->right = x+w;
	if (y + h > bounds->bottom)
		bounds->bottom = y+h;
}

static int find_best_matching_led (struct visible_led_info *led_points, int num_leds, struct blob *blob, double *out_sqerror)
{
	double best_z;
	int best_led_index = -1;
	double best_sqerror = 1e20;

	for (int i = 0; i < num_leds; i++) {
		struct visible_led_info *led_info = led_points + i;
		vec3f *pos = &led_info->pos;
		double led_radius = led_info->led_radius;
		double dx = abs(pos->x - blob->x);
		double dy = abs(pos->y - blob->y);
		double sqerror = dx*dx + dy*dy;

		/* Check if the LED falls within the bounding box
		 * has smaller error distance, or is closer to the camera (smaller Z) */
		if (sqerror < (led_radius*led_radius)) {
			if (best_led_index < 0 || sqerror < best_sqerror || best_z > pos->z) {
				best_z = pos->z;
				best_led_index = i;
				best_sqerror = sqerror;
			}
		}
	}

	if (out_sqerror)
		*out_sqerror = best_sqerror;
	return best_led_index;
}

static void
check_pose_prior(rift_pose_metrics *score, posef *pose, posef *pose_prior, const vec3f *pos_variance, const vec3f *rot_variance)
{
	quatf orient_diff;
	int i;

	score->match_flags |= RIFT_POSE_HAD_PRIOR;

	ovec3f_subtract(&pose->pos, &pose_prior->pos, &score->pos_error);

	oquatf_diff(&pose->orient, &pose_prior->orient, &orient_diff);
	oquatf_normalize_me(&orient_diff);
	oquatf_to_rotation(&orient_diff, &score->orient_error);

	/* Check each component of position and rotation are within the passed variance and
	 * clear any return flag that's not set */
	if (pos_variance) {
		score->match_flags |= ~RIFT_POSE_MATCH_POSITION;

		for (i = 0; i < 3; i++) {
			if (score->pos_error.arr[i] * score->pos_error.arr[i] > pos_variance->arr[i])
				score->match_flags &= ~RIFT_POSE_MATCH_POSITION;
		}
	}

	if (rot_variance) {
		score->match_flags |= RIFT_POSE_MATCH_ORIENT;

		for (i = 0; i < 3; i++) {
			if (score->orient_error.arr[i] * score->orient_error.arr[i] > rot_variance->arr[i])
				score->match_flags &= ~RIFT_POSE_MATCH_ORIENT;
		}
	}
}

void rift_evaluate_pose_with_prior (rift_pose_metrics *score, posef *pose,
	bool prior_must_match, posef *pose_prior, const vec3f *pos_variance, const vec3f *rot_variance,
	struct blob *blobs, int num_blobs,
	int device_id, rift_led *leds, int num_leds,
	dmat3 *camera_matrix, double dist_coeffs[5], bool dist_fisheye,
	rift_rect_t *out_bounds)
{
	/*
	 * 1. Project the LED points with the provided pose
	 * 2. Build a bounding box for the points
	 * 3. For blobs within the bounding box, see if they match a LED
	 * 4. Count up the matched LED<->Blob correspodences, and the reprojection error
	 */
	vec3f led_out_points[MAX_OBJECT_LEDS];
	struct visible_led_info visible_led_points[MAX_OBJECT_LEDS];
	
	bool first_visible = true;
	/* FIXME: Pass LED size in the model */
	double led_radius_mm = 5.0 / 1000.0;
	double focal_length;
	rift_rect_t bounds = { 0, };
	int i;
	
	assert (num_leds > 0);
	assert (num_blobs > 0);
	assert (score != NULL);

	score->match_flags = 0;
	score->reprojection_error = 0.0;
	score->matched_blobs = 0;
	score->unmatched_blobs = 0;
	score->visible_leds = 0;
	
	/* Project HMD LEDs into the distorted image space */
	rift_project_points(leds, num_leds,
	    camera_matrix, dist_coeffs, dist_fisheye,
	    pose, led_out_points);
	
	/* Compute LED pixel size based on model distance below
	 * using the larger X/Y focal length and LED's Z value */
	focal_length = OHMD_MAX(camera_matrix->m[0], camera_matrix->m[4]);
	
	/* Calculate the bounding box and visible LEDs */
	for (i = 0; i < num_leds; i++) {
		vec3f *p = led_out_points + i;
		
		/* FIXME: Don't hard code the screen size here */
		if (p->x < 0 || p->y < 0 || p->x > 1280 || p->y > 960)
		 continue; // Outside the visible screen space
		
		vec3f normal, position;
		double facing_dot;
		oquatf_get_rotated(&pose->orient, &leds[i].pos, &position);
		ovec3f_add (&pose->pos, &position, &position);
		
		ovec3f_normalize_me (&position);
		oquatf_get_rotated(&pose->orient, &leds[i].dir, &normal);
		facing_dot = ovec3f_get_dot (&position, &normal);
		
#if 0
		printf (" LED %u pos %f,%f,%f -> %f,%f (lin %f,%f,%f)\n",
			i, leds[i].pos.x, leds[i].pos.y, leds[i].pos.z, 
			p->x, p->y, position.x, position.y, position.z);
#endif

		if (facing_dot < -0.25) {
			double led_radius = 4.0;
			if (position.z > 0.0) {
				led_radius = focal_length * led_radius_mm / position.z;
			}

			visible_led_points[score->visible_leds].pos = *p;
			visible_led_points[score->visible_leds].led_radius = led_radius;
			score->visible_leds++;
			
			/* Expand the bounding box */
			if (first_visible) {
				bounds.left = p->x - led_radius;
				bounds.top = p->y - led_radius;
				bounds.right= p->x + 2 * led_radius;
				bounds.bottom = p->x + 2 * led_radius;
				first_visible = false;
			}
			else {
				expand_rect(&bounds, p->x - led_radius, p->y - led_radius, 2 * led_radius, 2 * led_radius);
			}
		}
	}

	if (score->visible_leds < 5)
		goto done;

	//printf ("Bounding box for pose is %f,%f -> %f,%f\n", bounds.left, bounds.top, bounds.right, bounds.bottom);
	
	/* Iterate the blobs and see which ones are within the bounding box and have a matching LED */
	for (i = 0; i < num_blobs; i++) {
		struct blob *b = blobs + i;
		int led_object_id = LED_OBJECT_ID (b->led_id);

		/* Skip blobs which already have an ID not belonging to this device */
		if (led_object_id != LED_INVALID_ID && led_object_id != device_id)
			continue;

		if (b->x >= bounds.left && b->y >= bounds.top &&
			b->x < bounds.right && b->y < bounds.bottom) {
			double sqerror;

			int match_led_index = find_best_matching_led (visible_led_points, score->visible_leds, b, &sqerror);
			if (match_led_index >= 0) {
				score->reprojection_error += sqerror;
				score->matched_blobs++;
			} else {
				score->unmatched_blobs++;
			}
		}
	}

	if (score->matched_blobs < 5)
		goto done;

	double error_per_led = score->reprojection_error / score->matched_blobs;

	/* At this point, we have at least 5 LEDs and their blobs matching */

	/* If we have a pose prior, calculate the rotation and translation error and match flags as needed */
	if (pose_prior) {
		check_pose_prior(score, pose, pose_prior, pos_variance, rot_variance);
	}

	if (POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_POSITION|RIFT_POSE_MATCH_ORIENT)) {
		if (error_per_led < 2.0 && (score->unmatched_blobs * 4 <= score->matched_blobs ||
		    (2 * score->visible_leds <= 3 * score->matched_blobs))) {
#if 0
				printf("Got good prior match within pos (%f, %f, %f) rot (%f, %f, %f)\n",
						pos_variance->x, pos_variance->y, pos_variance->z,
						rot_variance->x, rot_variance->y, rot_variance->z);
#endif
			score->match_flags |= RIFT_POSE_MATCH_GOOD;

			if (error_per_led < 1.5)
				score->match_flags |= RIFT_POSE_MATCH_STRONG;
		}
	}
	else if (prior_must_match) {
		/* If we must match the prior and failed, bail out */
		goto done;
	} else {
		/* If we matched all the blobs in the pose bounding box (allowing 25% noise / overlapping blobs)
		 * or if we matched a large proportion (2/3) of the LEDs we expect to be visible, then consider this a good pose match */
		if (score->visible_leds > 6 && score->matched_blobs > 6 &&
				error_per_led < 3.0 &&
				(score->unmatched_blobs * 4 <= score->matched_blobs || (2 * score->visible_leds <= 3 * score->matched_blobs))) {

			score->match_flags |= RIFT_POSE_MATCH_GOOD;

			/* If we had no pose prior, but a close reprojection error, allow a STRONG match */
			/* If we had a pose prior and got here, the pose is out of tolerance, so only permit a "GOOD" match */
			if (pose_prior == NULL && error_per_led < 1.5)
				score->match_flags |= RIFT_POSE_MATCH_STRONG;
		}
	}

#if 0
	printf ("score for pose is %u matched %u unmatched %u visible %f error\n",
		score->matched_blobs, score->unmatched_blobs, score->visible_leds, score->reprojection_error);
#endif

done:
	if (out_bounds)
		*out_bounds = bounds;
}

void rift_evaluate_pose (rift_pose_metrics *score, posef *pose,
	struct blob *blobs, int num_blobs,
	int device_id, rift_led *leds, int num_leds,
	dmat3 *camera_matrix, double dist_coeffs[5], bool dist_fisheye,
	rift_rect_t *out_bounds)
{
	rift_evaluate_pose_with_prior (score, pose, false, NULL, NULL, NULL,
	    blobs, num_blobs, device_id, leds, num_leds,
	    camera_matrix, dist_coeffs, dist_fisheye, out_bounds);
}

void rift_mark_matching_blobs (posef *pose,
	struct blob *blobs, int num_blobs,
	int device_id, rift_led *leds, int num_leds,
	dmat3 *camera_matrix, double dist_coeffs[5], bool dist_fisheye)
{
	/*
	 * 1. Project the LED points with the provided pose
	 * 2. Build a bounding box for the points
	 * 3. For blobs within the bounding box, see if they match a LED
	 * 4. Mark each blob with an ID based on the LED id and device ID
	 */
	vec3f led_out_points[MAX_OBJECT_LEDS];
	struct visible_led_info visible_led_points[MAX_OBJECT_LEDS];

	int num_visible_leds = 0;
	bool first_visible = true;
	/* FIXME: Pass LED size in the model */
	double led_radius_mm = 5.0 / 1000.0;
	double focal_length;
	rift_rect_t bounds = { 0, };
	int i;

	assert (num_leds > 0);
	assert (num_blobs > 0);

	/* Take the larger focal length for calculating pixel sizes. They are usually
	 * the same anyway */
	focal_length = OHMD_MAX(camera_matrix->m[0], camera_matrix->m[4]);

	/* Project HMD LEDs into the distorted image space */
	rift_project_points(leds, num_leds,
	    camera_matrix, dist_coeffs, dist_fisheye,
	    pose, led_out_points);

	/* Calculate the bounding box and visible LEDs */
	for (i = 0; i < num_leds; i++) {
		vec3f *p = led_out_points + i;

		/* FIXME: Don't hard code the screen size here */
		if (p->x < 0 || p->y < 0 || p->x > 1280 || p->y > 960)
		 continue; // Outside the visible screen space

		vec3f normal, position;
		double facing_dot;
		oquatf_get_rotated(&pose->orient, &leds[i].pos, &position);
		ovec3f_add (&pose->pos, &position, &position);

		ovec3f_normalize_me (&position);
		oquatf_get_rotated(&pose->orient, &leds[i].dir, &normal);
		facing_dot = ovec3f_get_dot (&position, &normal);

		LOGV("LED %d @ %f,%f dot %f\n", i, p->x, p->y, facing_dot);

		if (facing_dot < 0.65) {
			struct visible_led_info *led_info = visible_led_points + num_visible_leds;
			double led_radius = 5.0;
			if (position.z > 0.0) {
				led_radius = focal_length * led_radius_mm / position.z;
			}

			led_info->pos = *p;
			led_info->led_radius = led_radius;
			led_info->led = leds + i;

			num_visible_leds++;

			/* Expand the bounding box */
			if (first_visible) {
				bounds.left = p->x - led_radius;
				bounds.top = p->y - led_radius;
				bounds.right= p->x + 2 * led_radius;
				bounds.bottom = p->x + 2 * led_radius;
				first_visible = false;
			}
			else {
				expand_rect(&bounds, p->x - led_radius, p->y - led_radius, 2 * led_radius, 2 * led_radius);
			}
		}
	}

	//printf ("Bounding box for pose is %f,%f -> %f,%f\n", bounds.left, bounds.top, bounds.right, bounds.bottom);

	/* Iterate the blobs and see which ones are within the bounding box and have a matching LED */
	for (i = 0; i < num_blobs; i++) {
		struct blob *b = blobs + i;
		int led_object_id = LED_OBJECT_ID (b->led_id);

		/* Skip blobs which already have an ID not belonging to this device */
		if (led_object_id != LED_INVALID_ID && led_object_id != device_id)
			continue;

		if (b->x >= bounds.left && b->y >= bounds.top &&
			b->x < bounds.right && b->y < bounds.bottom) {

			int match_led_index = find_best_matching_led (visible_led_points, num_visible_leds, b, NULL);
			if (match_led_index >= 0) {
				struct visible_led_info *led_info = visible_led_points + match_led_index;
				rift_led *match_led = led_info->led;

				/* FIXME: Get the ID directly from the LED */
				int led_index = match_led - leds;
				b->prev_led_id = b->led_id;
				b->led_id = LED_MAKE_ID (device_id, led_index);
				if (b->led_id != b->prev_led_id)
					LOGV("Marking LED %d/%d at %f,%f now %d (was %d)\n", device_id, led_index, b->x, b->y, b->led_id, b->prev_led_id);
			}
			else {
					LOGV("No LED match %f,%f\n", b->x, b->y);
			}
		}
	}
}

/* Return true if new_score is for a more likely pose than old_score */
bool rift_score_is_better_pose (rift_pose_metrics *old_score, rift_pose_metrics *new_score)
{
	/* if our previous best pose was "strong", only take better "strong" poses */
	if (POSE_HAS_FLAGS(old_score, RIFT_POSE_MATCH_STRONG) && !POSE_HAS_FLAGS(new_score, RIFT_POSE_MATCH_STRONG))
		return false;

	/* If the old score wasn't any good, but the new one is - take the new one */
	if (!POSE_HAS_FLAGS(old_score, RIFT_POSE_MATCH_GOOD) && POSE_HAS_FLAGS(new_score, RIFT_POSE_MATCH_GOOD))
		return true;

	double new_error_per_led = new_score->reprojection_error / new_score->matched_blobs;
	double best_error_per_led = 10.0;

	if (old_score->matched_blobs > 0)
		best_error_per_led = old_score->reprojection_error / old_score->matched_blobs;

	if (old_score->matched_blobs < new_score->matched_blobs && (new_error_per_led < best_error_per_led))
		return true; /* prefer more matched blobs with tighter error/LED */

	if (old_score->matched_blobs+1 < new_score->matched_blobs && (new_error_per_led < best_error_per_led * 1.25))
		return true; /* prefer at least 2 more matched blobs with slightly worse error/LED */

	if (old_score->matched_blobs == new_score->matched_blobs &&
			new_score->reprojection_error < old_score->reprojection_error)
		return true; /* else, prefer closer reprojection with at least as many matches*/

	/* If both scores have pose priors, prefer the one where the orientation better matches the prior */
	if (POSE_HAS_FLAGS(old_score, RIFT_POSE_HAD_PRIOR) && POSE_HAS_FLAGS(new_score, RIFT_POSE_HAD_PRIOR)) {
		if (ovec3f_get_length(&new_score->orient_error) < ovec3f_get_length(&old_score->orient_error))
			return true;
	}

	return false;
}
