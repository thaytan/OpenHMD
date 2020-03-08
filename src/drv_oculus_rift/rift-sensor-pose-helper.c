// Copyright 2020 Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
#include <assert.h>
#include <stdio.h>

#include "rift-sensor-pose-helper.h"
#include "rift-sensor-opencv.h"

typedef struct {
 double left;
 double top;
 double right;
 double bottom;
} rect_t;

static void expand_rect(rect_t *bounds, double x, double y, double w, double h)
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

		/* Check if the LED falls within the bounding box */
		if (blob->width > 2 * (dx-led_radius) && blob->height > 2 * (dy-led_radius)) {
			double sqerror = dx*dx + dy*dy;
			if (best_led == NULL || sqerror < best_sqerror) {
				best_led = led;
				best_sqerror = sqerror;
			}
		}
	}

	*out_sqerror = best_sqerror;
	return best_led;
}

void rift_evaluate_pose (rift_pose_metrics *score, quatf *orient, vec3f *trans,
	struct blob *blobs, int num_blobs,
	rift_led *leds, int num_leds,
	dmat3 *camera_matrix, double dist_coeffs[5], bool dist_fisheye)
{
	/*
	 * 1. Project the LED points with the provided pose
	 * 2. Build a bounding box for the points
	 * 3. For blobs within the bounding box, see if they match a LED
	 * 4. Count up the matched LED<->Blob correspodences, and the reprojection error
	 */
	vec3f led_out_points[MAX_OBJECT_LEDS];
	vec3f visible_led_points[MAX_OBJECT_LEDS];
	
	int matched_blobs = 0;
	int unmatched_blobs = 0;
	int visible_leds = 0;
	double reprojection_error = 0.0;
	bool first_visible = true;
	double led_radius;
	bool good_pose_match = false;
	rect_t bounds = { 0, };
	int i;
	
	assert (num_leds > 0);
	assert (num_blobs > 0);
	
	/* Project HMD LEDs into the distorted image space */
	rift_project_points(leds, num_leds,
	    camera_matrix, dist_coeffs, dist_fisheye,
	    orient, trans, led_out_points);
	
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
		oquatf_get_rotated(orient, &leds[i].pos, &position);
		ovec3f_add (trans, &position, &position);
		
		ovec3f_normalize_me (&position);
		oquatf_get_rotated(orient, &leds[i].dir, &normal);
		facing_dot = ovec3f_get_dot (&position, &normal);
		
#if 0
		printf (" LED %u pos %f,%f,%f -> %f,%f (lin %f,%f,%f)\n",
			i, leds[i].pos.x, leds[i].pos.y, leds[i].pos.z, 
			p->x, p->y, position.x, position.y, position.z);
#endif

		if (facing_dot < -0.25) {
			visible_led_points[visible_leds] = *p;
			visible_leds++;
			
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
		if (b->x >= bounds.left && b->y >= bounds.top &&
			b->x < bounds.right && b->y < bounds.bottom) {
			double sqerror;

			vec3f *match_led = find_best_matching_led (visible_led_points, visible_leds, b, led_radius, &sqerror);
			if (match_led != NULL) {
				reprojection_error += sqerror;
				matched_blobs++;
			} else {
				unmatched_blobs++;
			}
		}
	}


  if (visible_leds > 4 && matched_blobs > 3) {
    /* If we matched all the blobs in the pose bounding box (allowing 25% noise / overlapping blobs)
     * or if we matched a large proportion (2/3) of the LEDs we expect to be visible, then consider this a good pose match */
    if (unmatched_blobs * 4 <= matched_blobs ||
        (2 * visible_leds <= 3 * matched_blobs)) {
			good_pose_match = true;
		}
	}

#if 0
	printf ("score for pose is %u matched %u unmatched %u visible %f error\n",
		matched_blobs, unmatched_blobs, visible_leds, reprojection_error);
#endif
	score->matched_blobs = matched_blobs;
	score->unmatched_blobs = unmatched_blobs;
	score->visible_leds = visible_leds;
	score->reprojection_error = reprojection_error;
	score->good_pose_match = good_pose_match;
}

void rift_mark_matching_blobs (quatf *orient, vec3f *trans,
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
	rect_t bounds = { 0, };
	int i;

	assert (num_leds > 0);
	assert (num_blobs > 0);

	/* Project HMD LEDs into the distorted image space */
	rift_project_points(leds, num_leds,
	    camera_matrix, dist_coeffs, dist_fisheye,
	    orient, trans, led_out_points);

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
		oquatf_get_rotated(orient, &leds[i].pos, &position);
		ovec3f_add (trans, &position, &position);

		ovec3f_normalize_me (&position);
		oquatf_get_rotated(orient, &leds[i].dir, &normal);
		facing_dot = ovec3f_get_dot (&position, &normal);

		if (facing_dot < -0.25) {
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

		/* Skip blobs which already have an ID */
		if (b->led_id >= 0)
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
				printf ("Marking LED %d/%d at %f,%f now %u\n", device_id, led_index, b->x, b->y, b->led_id);
			}
		}
	}
}
