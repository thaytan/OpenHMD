// Copyright 2020 Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
#include <assert.h>
#include <stdio.h>

#include "rift-sensor-pose-helper.h"
#include "rift-sensor-opencv.h"

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

static vec3f *find_best_matching_led (vec3f *led_points, int num_leds, struct blob *blob, double led_radius, double *out_sqerror)
{
  vec3f *best_led = NULL;
  double best_sqerror = 1e20;

	for (int i = 0; i < num_leds; i++) {
		vec3f *led = led_points + i;
		double dx = abs(led->x - blob->x);
		double dy = abs(led->y - blob->y);
		double sqerror = dx*dx + dy*dy;

		/* Check if the LED falls within the bounding box
		 * has smaller error distance, or is closer to the camera (smaller Z) */
		if (sqerror < led_radius * led_radius) {
			if (best_led == NULL || sqerror < best_sqerror || best_led->z > led->z) {
				best_led = led;
				best_sqerror = sqerror;
			}
		}
	}

	if (out_sqerror)
		*out_sqerror = best_sqerror;
	return best_led;
}

static bool
check_pose_prior(posef *pose, posef *pose_prior, const vec3f *pos_variance, const vec3f *rot_variance)
{
	vec3f pos_diff, orient_err;
	quatf orient_diff;
	int i;

	ovec3f_subtract(&pose->pos, &pose_prior->pos, &pos_diff);
	oquatf_diff(&pose->orient, &pose_prior->orient, &orient_diff);
	oquatf_to_rotation(&orient_diff, &orient_err);

	/* Check each component of position and rotation are within the passed variance */
	for (i = 0; i < 3; i++) {
		if (pos_diff.arr[i] * pos_diff.arr[i] > pos_variance->arr[i])
			return false;
		if (orient_err.arr[i] * orient_err.arr[i] > rot_variance->arr[i])
			return false;
	}

	return true;
}

void rift_evaluate_pose_with_prior (rift_pose_metrics *score, posef *pose,
	posef *pose_prior, const vec3f *pos_variance, const vec3f *rot_variance,
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
	vec3f visible_led_points[MAX_OBJECT_LEDS];
	
	int num_matched_blobs = 0;
	int num_unmatched_blobs = 0;
	int num_visible_leds = 0;
	double reprojection_error = 0.0;
	bool first_visible = true;
	double led_radius;
	bool good_pose_match = false;
	rift_rect_t bounds = { 0, };
	int i;
	
	assert (num_leds > 0);
	assert (num_blobs > 0);
	
	/* Project HMD LEDs into the distorted image space */
	rift_project_points(leds, num_leds,
	    camera_matrix, dist_coeffs, dist_fisheye,
	    pose, led_out_points);
	
	/* FIXME: Estimate LED size based on distance */
	led_radius = 5;
	
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
			visible_led_points[num_visible_leds] = *p;
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
	assert (bounds.top >= -led_radius);
	
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

			vec3f *match_led_pos = find_best_matching_led (visible_led_points, num_visible_leds, b, led_radius, &sqerror);
			if (match_led_pos != NULL) {
				reprojection_error += sqerror;
				num_matched_blobs++;
			} else {
				num_unmatched_blobs++;
			}
		}
	}

  if (num_visible_leds > 3 && num_matched_blobs > 3) {
    /* If we matched all the blobs in the pose bounding box (allowing 25% noise / overlapping blobs)
     * or if we matched a large proportion (2/3) of the LEDs we expect to be visible, then consider this a good pose match */
    if (num_unmatched_blobs * 4 <= num_matched_blobs ||
        (2 * num_visible_leds <= 3 * num_matched_blobs)) {
			good_pose_match = true;
		}
    else if (pose_prior) {
			/* We have a pose prior, see if it's a close match for the passed pose */
			bool have_close_prior = check_pose_prior(pose, pose_prior, pos_variance, rot_variance);

			if (have_close_prior && num_matched_blobs >= 2) {
				printf("Got good prior match within pos (%f, %f, %f) rot (%f, %f, %f)\n",
						pos_variance->x, pos_variance->y, pos_variance->z,
						rot_variance->x, rot_variance->y, rot_variance->z);
				good_pose_match = true;
			}
    }
	}

#if 0
	printf ("score for pose is %u matched %u unmatched %u visible %f error\n",
		num_matched_blobs, num_unmatched_blobs, num_visible_leds, reprojection_error);
#endif

	if (score) {
		score->matched_blobs = num_matched_blobs;
		score->unmatched_blobs = num_unmatched_blobs;
		score->visible_leds = num_visible_leds;
		score->reprojection_error = reprojection_error;
		score->good_pose_match = good_pose_match;
	}

	if (out_bounds)
		*out_bounds = bounds;
}

void rift_evaluate_pose (rift_pose_metrics *score, posef *pose,
	struct blob *blobs, int num_blobs,
	int device_id, rift_led *leds, int num_leds,
	dmat3 *camera_matrix, double dist_coeffs[5], bool dist_fisheye,
	rift_rect_t *out_bounds)
{
  rift_evaluate_pose_with_prior (score, pose, NULL, NULL, NULL,
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
	vec3f visible_led_points[MAX_OBJECT_LEDS];
	rift_led *visible_leds[MAX_OBJECT_LEDS];

	int num_visible_leds = 0;
	bool first_visible = true;
	double led_radius;
	rift_rect_t bounds = { 0, };
	int i;

	assert (num_leds > 0);
	assert (num_blobs > 0);

	/* Project HMD LEDs into the distorted image space */
	rift_project_points(leds, num_leds,
	    camera_matrix, dist_coeffs, dist_fisheye,
	    pose, led_out_points);

	/* FIXME: Estimate LED size based on distance */
	led_radius = 10;

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
			visible_led_points[num_visible_leds] = *p;
			visible_leds[num_visible_leds] = leds + i;
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
	assert (bounds.top >= -led_radius);

	/* Iterate the blobs and see which ones are within the bounding box and have a matching LED */
	for (i = 0; i < num_blobs; i++) {
		struct blob *b = blobs + i;
		int led_object_id = LED_OBJECT_ID (b->led_id);

		/* Skip blobs which already have an ID not belonging to this device */
		if (led_object_id != LED_INVALID_ID && led_object_id != device_id)
			continue;

		if (b->x >= bounds.left && b->y >= bounds.top &&
			b->x < bounds.right && b->y < bounds.bottom) {

			vec3f *match_led_pos = find_best_matching_led (visible_led_points, num_visible_leds, b, led_radius, NULL);
			if (match_led_pos != NULL) {
				int index = match_led_pos - visible_led_points;
				rift_led *match_led = visible_leds[index];

				/* FIXME: Get the ID directly from the LED */
				int led_index = match_led - leds;
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
