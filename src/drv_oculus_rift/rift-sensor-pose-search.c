/*
 * Rift sensor interface
 * Copyright 2014-2015 Philipp Zabel
 * Copyright 2019 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */

#define _GNU_SOURCE
#define __STDC_FORMAT_MACROS

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <inttypes.h>

#include "rift-sensor.h"
#include "rift-sensor-device.h"
#include "rift-sensor-pose-search.h"

#include "rift-sensor-blobwatch.h"

#include "rift-sensor-maths.h"
#include "rift-sensor-opencv.h"

static void update_device_and_blobs (rift_pose_finder *pf, rift_sensor_analysis_frame *frame,
	rift_tracked_device *dev, rift_sensor_frame_device_state *dev_state,
	rift_tracked_device_exposure_info *exp_dev_info,
	posef *obj_cam_pose);

void rift_pose_finder_init(rift_pose_finder *pf, rift_sensor_camera_params *calib,
		rift_pose_finder_cb pose_cb, void *pose_cb_data)
{
	pf->have_camera_pose = false;
	pf->calib = calib;
	pf->cs = correspondence_search_new(calib);

  pf->pose_cb = pose_cb;
  pf->pose_cb_data = pose_cb_data;
}

void rift_pose_finder_clear(rift_pose_finder *pf)
{
	correspondence_search_free(pf->cs);
}

void rift_pose_finder_process_blobs_fast(rift_pose_finder *pf,
        rift_sensor_analysis_frame *frame,
        rift_tracked_device **devs)
{
	rift_tracker_exposure_info *exposure_info = &frame->exposure_info;
	blobservation* bwobs = frame->bwobs;
	int d;

	/* Only process the devices that were available when this frame was captured */
	for (d = 0; d < frame->n_devices; d++) {
		rift_tracked_device *dev = devs[d];
		rift_sensor_frame_device_state *dev_state = frame->capture_state + d;
		rift_tracked_device_exposure_info *exp_dev_info = exposure_info->devices + d;
		posef obj_world_pose, obj_cam_pose;

		/* Get the latest pose estimate for this device */
		if (!rift_tracked_device_get_latest_exposure_info_pose(dev, exp_dev_info)) {
			LOGV ("Skipping fast analysis of device %d. No fusion slot assigned\n", d);
			continue;
		}

		obj_world_pose = dev_state->capture_world_pose;

		LOGV ("Sensor %d Fusion provided pose for device %d, %f %f %f %f pos %f %f %f "
			"pos_err %f %f %f rot_err %f %f %f had_pose_lock %d",
			pf->sensor_id, dev->id,
			obj_world_pose.orient.x, obj_world_pose.orient.y, obj_world_pose.orient.z, obj_world_pose.orient.w,
			obj_world_pose.pos.x, obj_world_pose.pos.y, obj_world_pose.pos.z,
			exp_dev_info->pos_error.x, exp_dev_info->pos_error.y, exp_dev_info->pos_error.z,
			exp_dev_info->rot_error.x, exp_dev_info->rot_error.y, exp_dev_info->rot_error.z,
			exp_dev_info->had_pose_lock);

		/* If we don't have a camera pose yet, skip straight to correspondence
		 * search */
		if (!pf->have_camera_pose) {
			frame->need_long_analysis = true;
			continue;
		}

		oposef_apply_inverse(&obj_world_pose, &pf->camera_pose, &obj_cam_pose);

		LOGD ("Sensor %d Frame %d searching for matching pose for device %d, initial quat %f %f %f %f pos %f %f %f",
			pf->sensor_id, frame->id, dev->id,
			obj_cam_pose.orient.x, obj_cam_pose.orient.y, obj_cam_pose.orient.z, obj_cam_pose.orient.w,
			obj_cam_pose.pos.x, obj_cam_pose.pos.y, obj_cam_pose.pos.z);

		dev_state->final_cam_pose = obj_cam_pose;

		rift_evaluate_pose_with_prior(&dev_state->score, &obj_cam_pose,
			true, &obj_cam_pose, &exp_dev_info->pos_error, &exp_dev_info->rot_error,
			frame->bwobs->blobs, frame->bwobs->num_blobs,
			dev->id, dev->leds->points, dev->leds->num_points,
			pf->calib, NULL);

		if (POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_GOOD)) {
			LOGD("Sensor %d already had good pose match for device %d matched %u blobs of %u",
				pf->sensor_id, dev->id, dev_state->score.matched_blobs, dev_state->score.visible_leds);
		} else {
			int num_blobs = 0;

			/* See if we still have enough labelled blobs to try to re-acquire the pose without a
			 * full search */
			for (int index = 0; index < bwobs->num_blobs; index++) {
				struct blob *b = bwobs->blobs + index;
				if (LED_OBJECT_ID (b->led_id) == dev->id) {
						num_blobs++;
				}
			}

			if (num_blobs > 4) {
				estimate_initial_pose (bwobs->blobs, bwobs->num_blobs,
					dev->id, dev->leds->points, dev->leds->num_points, pf->calib,
					&obj_cam_pose, NULL, NULL, true);

				rift_evaluate_pose_with_prior(&dev_state->score, &obj_cam_pose,
					true, &dev_state->final_cam_pose, &exp_dev_info->pos_error, &exp_dev_info->rot_error,
					bwobs->blobs, bwobs->num_blobs,
					dev->id, dev->leds->points, dev->leds->num_points, pf->calib,
					NULL);

				if (POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_GOOD)) {
					LOGD("Sensor %d re-acquired match for device %d matched %u blobs of %u",
						pf->sensor_id, dev->id, dev_state->score.matched_blobs, dev_state->score.visible_leds);
				}
#if LOGLEVEL == 0
				else {
					quatf orient_diff;
					vec3f pos_error, orient_error;

					ovec3f_subtract(&obj_cam_pose.pos, &dev_state->final_cam_pose.pos, &pos_error);

					oquatf_diff(&obj_cam_pose.orient, &dev_state->final_cam_pose.orient, &orient_diff);
					oquatf_normalize_me(&orient_diff);
					oquatf_to_rotation(&orient_diff, &orient_error);

					LOGD("Sensor %d device %d had %d prior blobs, but failed match. Yielded pose %f %f %f %f pos %f %f %f (match %d of %d) rot_error %f %f %f pos_error %f %f %f",
						pf->sensor_id, dev->id, num_blobs,
						obj_cam_pose.orient.x, obj_cam_pose.orient.y, obj_cam_pose.orient.z, obj_cam_pose.orient.w,
						obj_cam_pose.pos.x, obj_cam_pose.pos.y, obj_cam_pose.pos.z,
						dev_state->score.matched_blobs, dev_state->score.visible_leds,
						orient_error.x, orient_error.y, orient_error.z,
						pos_error.x, pos_error.y, pos_error.z);
				}
#endif
			}
		}

		/* If we got at least a good match, pass it on for consideration by the fusion */
		if (POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_GOOD)) {
			update_device_and_blobs (pf, frame, dev, dev_state, exp_dev_info, &obj_cam_pose);
		} else {
			/* Didn't find this device - send the frame for long analysis */
			LOGD("Sensor %d frame %d needs full search for device %d - sending to long analysis thread",
				pf->sensor_id, frame->id, dev->id);
			frame->need_long_analysis = true;
			continue;
		}
	}
}

void rift_pose_finder_process_blobs_long(rift_pose_finder *pf, rift_sensor_analysis_frame *frame,
        rift_tracked_device **devs)
{
	rift_tracker_exposure_info *exposure_info = &frame->exposure_info;
	blobservation* bwobs = frame->bwobs;
	int d, pass;
	bool had_strong_matches = false;

	LOGD("Sensor %d Frame %d - starting long search for devices", pf->sensor_id, frame->id);

	correspondence_search_set_blobs (pf->cs, bwobs->blobs, bwobs->num_blobs);

	for (pass = 0; pass < 2; pass++) {
		/* Only process the devices that were available when this frame was captured */
		for (d = 0; d < frame->n_devices; d++) {
			rift_tracked_device *dev = devs[d];
			rift_sensor_frame_device_state *dev_state = frame->capture_state + d;
			rift_tracked_device_exposure_info *exp_dev_info = exposure_info->devices + d;
			posef obj_cam_pose;
			bool do_aligned_checks = false;
			float pose_tolerance = 0.0;
			CorrespondenceSearchFlags search_flags = CS_FLAG_STOP_FOR_STRONG_MATCH;

			if (dev_state->found_device_pose)
				continue; /* We already found a pose for this device */

			if (!rift_tracked_device_get_latest_exposure_info_pose(dev, exp_dev_info)) {
				LOGV ("Skipping long analysis of device %d. No fusion slot assigned\n", d);
				continue;
			}

			if (pass == 0) {
				if (dev->id == 0)
					search_flags |= CS_FLAG_MATCH_ALL_BLOBS; /* Let the HMD match whatever it can on the first pass */

				search_flags |= CS_FLAG_SHALLOW_SEARCH;
			} else
				search_flags |= CS_FLAG_DEEP_SEARCH;

			if (exp_dev_info->fusion_slot == -1) {
				LOGV ("Sensor %d Skipping long analysis of device %d. No fusion slot assigned\n", pf->sensor_id, d);
				continue;
			}

			/* If the gravity vector error standard deviation is small enough, try for an
			 * aligned pose from the prior, within 2 standard deviation */
			if (pf->have_camera_pose) {
				if (dev_state->gravity_error_rad < DEG_TO_RAD(30)) {
					search_flags |= CS_FLAG_MATCH_GRAVITY;
					do_aligned_checks = true;
					pose_tolerance = OHMD_MAX(2 * dev_state->gravity_error_rad, DEG_TO_RAD(10));
				}

				oposef_apply_inverse(&dev_state->capture_world_pose, &pf->camera_pose, &obj_cam_pose);

				search_flags |= CS_FLAG_HAVE_POSE_PRIOR;
			}

			if (POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_GOOD) && had_strong_matches) {
				/* We have a good pose match for this device, that we found on the first
				 * pass. If any other device found a strong match though, then that may
				 * have claimed blobs we were relying on  - so re-check our pose and possibly start again */
				if (search_flags & CS_FLAG_HAVE_POSE_PRIOR) {
					rift_evaluate_pose_with_prior(&dev_state->score, &obj_cam_pose,
						do_aligned_checks, &dev_state->final_cam_pose, &exp_dev_info->pos_error, &exp_dev_info->rot_error,
						bwobs->blobs, bwobs->num_blobs,
						dev->id, dev->leds->points, dev->leds->num_points,
						pf->calib, NULL);
				}
				else {
					rift_evaluate_pose (&dev_state->score, &obj_cam_pose,
						frame->bwobs->blobs, frame->bwobs->num_blobs,
						dev->id, dev->leds->points, dev->leds->num_points,
						pf->calib, NULL);
				}

				/* If we don't have a good match any more, do another shallow search */
				if (!POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_GOOD))
					search_flags |= CS_FLAG_SHALLOW_SEARCH;
			}

			if (search_flags & CS_FLAG_DEEP_SEARCH) {
				LOGD("Sensor %d deep search for device %d\n", pf->sensor_id, dev->id);
			}

			if (correspondence_search_find_one_pose (pf->cs, dev->id, search_flags, &obj_cam_pose,
					&exp_dev_info->pos_error, &exp_dev_info->rot_error,
					&pf->cam_gravity_vector, pose_tolerance, &dev_state->score)) {
				if (do_aligned_checks) {
					LOGD("Got aligned pose %f, %f, %f, %f for device %d with tolerance %f!",
						obj_cam_pose.orient.x, obj_cam_pose.orient.y, obj_cam_pose.orient.z, obj_cam_pose.orient.w,
						d, RAD_TO_DEG(pose_tolerance));
				}
			}
			else if (do_aligned_checks) {
				LOGD("No aligned pose for device %d with tolerance %f!", d, RAD_TO_DEG(pose_tolerance));
			}

			LOGV("Sensor %d Frame %d - doing long search with flags 0x%x for device %d matched %u blobs of %u from %d (%s match)",
				pf->sensor_id, frame->id, search_flags, dev->id,
				dev_state->score.matched_blobs, dev_state->score.visible_leds, bwobs->num_blobs,
				POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_GOOD) ? "good" : "bad");

			/* Require a strong pose match on the quick loop */
			if (pass == 0 && !POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_STRONG))
				continue;

			if (POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_GOOD)) {
				had_strong_matches |= POSE_HAS_FLAGS(&dev_state->score, RIFT_POSE_MATCH_STRONG);

				update_device_and_blobs (pf, frame, dev, dev_state, exp_dev_info, &obj_cam_pose);
				frame->long_analysis_found_new_blobs = true;
			}
		}
	}
}

static void
update_device_and_blobs (rift_pose_finder *pf, rift_sensor_analysis_frame *frame,
	rift_tracked_device *dev, rift_sensor_frame_device_state *dev_state,
	rift_tracked_device_exposure_info *exp_dev_info,
	posef *obj_cam_pose)
{
	blobservation* bwobs = frame->bwobs;

	/* Clear existing blob IDs for this device, then
	 * back project LED ids into blobs if we find them and the dot product
	 * shows them pointing strongly to the camera */
	rift_clear_blob_labels (bwobs->blobs, bwobs->num_blobs, dev->id);

	rift_mark_matching_blobs (obj_cam_pose, bwobs->blobs, bwobs->num_blobs, dev->id,
			dev->leds->points, dev->leds->num_points, pf->calib);

	/* Refine the pose with PnP now that we've labelled the blobs */
	estimate_initial_pose (bwobs->blobs, bwobs->num_blobs,
		dev->id, dev->leds->points, dev->leds->num_points, pf->calib,
		obj_cam_pose, NULL, NULL, true);

	/* And label the blobs again in case we collected any more */
	rift_mark_matching_blobs (obj_cam_pose, bwobs->blobs, bwobs->num_blobs, dev->id,
			dev->leds->points, dev->leds->num_points, pf->calib);

	dev_state->final_cam_pose = *obj_cam_pose;

	LOGD ("sensor %d PnP for device %d yielded quat %f %f %f %f pos %f %f %f",
		pf->sensor_id, dev->id, dev_state->final_cam_pose.orient.x, dev_state->final_cam_pose.orient.y, dev_state->final_cam_pose.orient.z, dev_state->final_cam_pose.orient.w,
		dev_state->final_cam_pose.pos.x, dev_state->final_cam_pose.pos.y, dev_state->final_cam_pose.pos.z);

	posef pose = dev_state->final_cam_pose;
	posef *capture_pose = &dev_state->capture_world_pose;
	rift_pose_metrics *score = &dev_state->score;

	if (pf->have_camera_pose) {
		posef obj_cam_pose_prior;

		oposef_apply_inverse(&dev_state->capture_world_pose, &pf->camera_pose, &obj_cam_pose_prior);

		rift_evaluate_pose_with_prior(&dev_state->score, &pose,
			false, &obj_cam_pose_prior, &exp_dev_info->pos_error, &exp_dev_info->rot_error,
			frame->bwobs->blobs, frame->bwobs->num_blobs,
			dev->id, dev->leds->points, dev->leds->num_points,
			pf->calib, NULL);
	} else {
		rift_evaluate_pose (score, &pose,
			frame->bwobs->blobs, frame->bwobs->num_blobs,
			dev->id, dev->leds->points, dev->leds->num_points,
			pf->calib, NULL);
	}

	if (POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_GOOD)) {
		LOGV("Sensor %d Found %s pose match - %u LEDs matched %u visible ones flags %x\n",
			pf->sensor_id, POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_STRONG) ? "strong" : "good",
			score->matched_blobs, score->visible_leds, score->match_flags);

		if (pf->have_camera_pose) {
			/* The pose we found is the transform from object coords to camera-relative coords.
			 * Our camera pose stores the transform from camera to world, and what we
			 * need to give the fusion is transform from object->world.
			 *
			 * To get the transform from object->world, take object->camera pose, and apply
			 * the camera->world pose. */
			oposef_apply(&pose, &pf->camera_pose, &pose);

			assert (pf->pose_cb != NULL);

			dev_state->found_device_pose = pf->pose_cb (pf->pose_cb_data, dev, frame, &pose, score);
		}
		/* Arbitrary 25 degree threshold for gravity vector matches the minimum error */
		else if (dev->id == 0 && oquatf_get_length (&capture_pose->orient) > 0.9 && dev_state->gravity_error_rad <= DEG_TO_RAD(25.0)) {
			/* No camera pose yet. If this is the HMD, we had an IMU pose at capture time,
			 * and the fusion has a good gravity vector from the IMU, use it to
			 * initialise the camera (world->camera) pose using the current headset pose.
			 * Calculate the xform from camera->world by applying
			 * the observed pose (object->camera), inverted (so camera->object) to our found
			 * fusion pose (object->world) to yield camera->world xform
			 */
			posef camera_object_pose = pose;
			oposef_inverse(&camera_object_pose);

			oposef_apply(&camera_object_pose, capture_pose, &pf->camera_pose);

			LOGI("Set sensor %d pose from device %d - tracker pose quat %f %f %f %f  pos %f %f %f"
					" fusion pose quat %f %f %f %f  pos %f %f %f gravity error %f degrees yielded"
					" world->camera pose quat %f %f %f %f  pos %f %f %f",
				pf->sensor_id, dev->id,
				pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w, pose.pos.x, pose.pos.y, pose.pos.z,
				capture_pose->orient.x, capture_pose->orient.y, capture_pose->orient.z, capture_pose->orient.w,
				capture_pose->pos.x, capture_pose->pos.y, capture_pose->pos.z, RAD_TO_DEG(dev_state->gravity_error_rad),
				pf->camera_pose.orient.x, pf->camera_pose.orient.y, pf->camera_pose.orient.z, pf->camera_pose.orient.w,
				pf->camera_pose.pos.x, pf->camera_pose.pos.y, pf->camera_pose.pos.z);

			const vec3f gravity_vector = {{ 0.0, 1.0, 0.0 }};

			quatf cam_orient = pf->camera_pose.orient;
			oquatf_inverse(&cam_orient);
			oquatf_get_rotated(&cam_orient, &gravity_vector, &pf->cam_gravity_vector);

			pf->have_camera_pose = true;
		}
		else if (dev->id == 0) {
			LOGD("Sensor %d No camera pose yet - gravity error is %f degrees rot_error (%f, %f, %f)",
			    pf->sensor_id, RAD_TO_DEG(dev_state->gravity_error_rad),
			    exp_dev_info->rot_error.x, exp_dev_info->rot_error.y, exp_dev_info->rot_error.z);
		}

	}
	else {
		LOGV("Sensor %d Failed pose match - only %u LEDs matched %u visible ones",
			pf->sensor_id, score->matched_blobs, score->visible_leds);
		rift_clear_blob_labels (bwobs->blobs, bwobs->num_blobs, dev->id);
	}
}

