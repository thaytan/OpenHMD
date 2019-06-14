/*
 * Pose estimation using OpenCV
 * Copyright 2015 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+ or BSL-1.0
 */
#include <opencv2/calib3d/calib3d.hpp>

extern "C" {
#include <stdio.h>

#include "rift-sensor-opencv.h"
#include "rift-sensor-blobwatch.h"
}


extern "C" bool estimate_initial_pose(struct blob *blobs, int num_blobs,
    rift_led *leds, int num_led_pos,
    dmat3 *camera_matrix, double dist_coeffs[4],
    dquat *rot, dvec3 *trans, bool use_extrinsic_guess)
{
	int i, j;
	int num_leds = 0;
	uint64_t taken = 0;
	int flags = CV_ITERATIVE;
	cv::Mat inliers;
	int iterationsCount = 50;
	float reprojectionError = 1.0;
	float confidence = 0.95;
	cv::Mat fishK = cv::Mat(3, 3, CV_64FC1, camera_matrix->m);
	cv::Mat fishDistCoeffs = cv::Mat(4, 1, CV_64FC1, dist_coeffs);
	cv::Mat dummyK = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat dummyD = cv::Mat::zeros(4, 1, CV_64FC1);
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec = cv::Mat(3, 1, CV_64FC1, (double *)trans);
	cv::Mat R = cv::Mat(3, 3, CV_64FC1, (double *)rot);
	cv::Rodrigues(R, rvec);

	/* count identified leds */
	for (i = 0; i < num_blobs; i++) {
		if (blobs[i].led_id < 0)
			continue;
		if (taken & (1ULL << blobs[i].led_id))
			continue;
		taken |= (1ULL << blobs[i].led_id);
		num_leds++;
	}

	if (num_leds < 4)
		return false;

	std::vector<cv::Point3f> list_points3d(num_leds);
	std::vector<cv::Point2f> list_points2d(num_leds);
	std::vector<cv::Point2f> list_points2d_undistorted(num_leds);

	taken = 0;
	for (i = 0, j = 0; i < num_blobs && j < num_leds; i++) {
		if (blobs[i].led_id < 0)
			continue;
		if (taken & (1ULL << blobs[i].led_id))
			continue;
		taken |= (1ULL << blobs[i].led_id);
		list_points3d[j].x = leds[blobs[i].led_id].pos.x;
		list_points3d[j].y = leds[blobs[i].led_id].pos.y;
		list_points3d[j].z = leds[blobs[i].led_id].pos.z;
		list_points2d[j].x = blobs[i].x;
		list_points2d[j].y = blobs[i].y;
		j++;
		printf ("LED %d at %d,%d (3D %f %f %f)\n",
		    blobs[i].led_id, blobs[i].x, blobs[i].y,
		    leds[blobs[i].led_id].pos.x,
		    leds[blobs[i].led_id].pos.y,
		    leds[blobs[i].led_id].pos.z);
	}

	num_leds = j;
	if (num_leds < 4)
		return false;
	list_points3d.resize(num_leds);
	list_points2d.resize(num_leds);
	list_points2d_undistorted.resize(num_leds);

	// we have distortion params for the openCV fisheye model
	// so we undistort the image points manually before passing them to the PnpRansac solver
	// and we give the solver identity camera + null distortion matrices
	cv::fisheye::undistortPoints(list_points2d, list_points2d_undistorted, fishK, fishDistCoeffs);

	cv::solvePnPRansac(list_points3d, list_points2d_undistorted, dummyK, dummyD, rvec, tvec,
			   use_extrinsic_guess, iterationsCount, reprojectionError,
			   confidence, inliers, flags);

	dvec3 v;
	double angle = sqrt(rvec.dot(rvec));
	double inorm = 1.0f / angle;

	v.x = rvec.at<double>(0) * inorm;
	v.y = rvec.at<double>(1) * inorm;
	v.z = rvec.at<double>(2) * inorm;
	dquat_from_axis_angle(rot, &v, angle);

	printf ("Got PnP pose quat %f %f %f %f  pos %f %f %f\n",
	     rot->x, rot->y, rot->z, rot->w,
       trans->x, trans->y, trans->z);
  return true;
}
