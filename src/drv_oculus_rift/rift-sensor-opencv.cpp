/*
 * Pose estimation using OpenCV
 * Copyright 2015 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+ or BSL-1.0
 */
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#if CV_MAJOR_VERSION >= 4
#include <opencv2/calib3d/calib3d_c.h>
#endif
#include <iostream>

using namespace std;

extern "C" {
#include <stdio.h>

#include "rift-sensor-opencv.h"
#include "rift-sensor-blobwatch.h"
}

static void
quatf_to_3x3 (cv::Mat &mat, quatf *me) {
        mat.at<double>(0,0) = 1 - 2 * me->y * me->y - 2 * me->z * me->z;
        mat.at<double>(0,1) =     2 * me->x * me->y - 2 * me->w * me->z;
        mat.at<double>(0,2) =     2 * me->x * me->z + 2 * me->w * me->y;

        mat.at<double>(1,0) =     2 * me->x * me->y + 2 * me->w * me->z;
        mat.at<double>(1,1) = 1 - 2 * me->x * me->x - 2 * me->z * me->z;
        mat.at<double>(1,2) =     2 * me->y * me->z - 2 * me->w * me->x;

        mat.at<double>(2,0) =     2 * me->x * me->z - 2 * me->w * me->y;
        mat.at<double>(2,1) =     2 * me->y * me->z + 2 * me->w * me->x;
        mat.at<double>(2,2) = 1 - 2 * me->x * me->x - 2 * me->y * me->y;
}

extern "C" bool estimate_initial_pose(struct blob *blobs, int num_blobs,
    int device_id, rift_led *leds, int num_led_pos,
    dmat3 *camera_matrix, double dist_coeffs[5], bool dist_fisheye,
    quatf *rot, vec3f *trans, int *num_leds_out,
    int *num_inliers, bool use_extrinsic_guess)
{
	int i, j;
	int num_leds = 0;
	uint64_t taken = 0;
	int flags = CV_ITERATIVE;
	cv::Mat inliers;
	int iterationsCount = 50;
	float confidence = 0.95;
	cv::Mat cameraK = cv::Mat(3, 3, CV_64FC1, camera_matrix->m);
	cv::Mat distCoeffs;
	cv::Mat dummyK = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat dummyD = cv::Mat::zeros(4, 1, CV_64FC1);
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);

	for (i = 0; i < 3; i++)
		tvec.at<double>(i) = trans->arr[i];

	quatf_to_3x3 (R, rot);
	cv::Rodrigues(R, rvec);

	//cout << "R = " << R << ", rvec = " << rvec << endl;

	/* count identified leds */
	for (i = 0; i < num_blobs; i++) {
		int led_id = blobs[i].led_id;
		if (LED_OBJECT_ID(led_id) != device_id)
			continue; /* invalid or LED id for another object */
		led_id = LED_LOCAL_ID(led_id);

		if (taken & (1ULL << led_id))
			continue;
		taken |= (1ULL << led_id);
		num_leds++;
	}
	if (num_leds_out)
		*num_leds_out = num_leds;

	if (num_leds < 4)
		return false;

	std::vector<cv::Point3f> list_points3d(num_leds);
	std::vector<cv::Point2f> list_points2d(num_leds);
	std::vector<cv::Point2f> list_points2d_undistorted(num_leds);

	taken = 0;
	for (i = 0, j = 0; i < num_blobs && j < num_leds; i++) {
		int led_id = blobs[i].led_id;
		if (LED_OBJECT_ID(led_id) != device_id)
			continue; /* invalid or LED id for another object */
		led_id = LED_LOCAL_ID(led_id);
		if (taken & (1ULL << led_id))
			continue;
		taken |= (1ULL << led_id);
		list_points3d[j].x = leds[led_id].pos.x;
		list_points3d[j].y = leds[led_id].pos.y;
		list_points3d[j].z = leds[led_id].pos.z;
		list_points2d[j].x = blobs[i].x;
		list_points2d[j].y = blobs[i].y;
		j++;

		LOGD ("LED %d at %f,%f (3D %f %f %f)\n",
		    blobs[i].led_id, blobs[i].x, blobs[i].y,
		    leds[led_id].pos.x,
		    leds[led_id].pos.y,
		    leds[led_id].pos.z);
	}

	num_leds = j;
	if (num_leds < 4)
		return false;
	list_points3d.resize(num_leds);
	list_points2d.resize(num_leds);
	list_points2d_undistorted.resize(num_leds);

  /* Need to support 2 different distortion models. Fisheye for CV1, Pinhole for DK2 */
  if (dist_fisheye) {
    distCoeffs = cv::Mat(4, 1, CV_64FC1, dist_coeffs);

		// we have distortion params for the openCV fisheye model
		// so we undistort the image points manually before passing them to the PnpRansac solver
		// and we give the solver identity camera + null distortion matrices
		cv::fisheye::undistortPoints(list_points2d, list_points2d_undistorted, cameraK, distCoeffs);
  }
  else {
    distCoeffs = cv::Mat(5, 1, CV_64FC1, dist_coeffs);
		// Pinhole camera undistort
    cv::undistortPoints(list_points2d, list_points2d_undistorted, cameraK, distCoeffs);
  }

	float reprojectionError = 2.0 / camera_matrix->m[0];

	cv::solvePnPRansac(list_points3d, list_points2d_undistorted, dummyK, dummyD, rvec, tvec,
			   use_extrinsic_guess, iterationsCount, reprojectionError,
			   confidence, inliers, flags);

	if (num_inliers)
		*num_inliers = inliers.rows;

	vec3f v;
	double angle = sqrt(rvec.dot(rvec));
	double inorm = 1.0f / angle;

	v.x = rvec.at<double>(0) * inorm;
	v.y = rvec.at<double>(1) * inorm;
	v.z = rvec.at<double>(2) * inorm;
	oquatf_init_axis (rot, &v, angle);

	for (i = 0; i < 3; i++)
		trans->arr[i] = tvec.at<double>(i);

	LOGV ("Got PnP pose quat %f %f %f %f  pos %f %f %f\n",
	     rot->x, rot->y, rot->z, rot->w,
	     trans->x, trans->y, trans->z);
	return true;
}

void rift_project_points(rift_led *leds, int num_led_pos,
		dmat3 *camera_matrix, double dist_coeffs[5], bool dist_fisheye,
		quatf *rot, vec3f *trans, vec3f *out_points)
{
	cv::Mat cameraK = cv::Mat(3, 3, CV_64FC1, camera_matrix->m);
	cv::Mat distCoeffs;

	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);
  int i;

	for (i = 0; i < 3; i++)
		tvec.at<double>(i) = trans->arr[i];

	quatf_to_3x3 (R, rot);
	cv::Rodrigues(R, rvec);

	std::vector<cv::Point3f> led_points3d(num_led_pos);
	std::vector<cv::Point2f> projected_points2d(num_led_pos);

	for (int i = 0; i < num_led_pos; i++) {
		led_points3d[i].x = leds[i].pos.x;
		led_points3d[i].y = leds[i].pos.y;
		led_points3d[i].z = leds[i].pos.z;
	}

  if (dist_fisheye) {
	  distCoeffs = cv::Mat(4, 1, CV_64FC1, dist_coeffs);
	  cv::fisheye::projectPoints (led_points3d, projected_points2d, rvec, tvec, cameraK, distCoeffs);
  }
  else {
    distCoeffs = cv::Mat(5, 1, CV_64FC1, dist_coeffs);
		// Pinhole camera undistort
    cv::projectPoints (led_points3d, rvec, tvec, cameraK, distCoeffs, projected_points2d);
  }

	for (int i = 0; i < num_led_pos; i++) {
		out_points[i].x = projected_points2d[i].x;
		out_points[i].y = projected_points2d[i].y;
		out_points[i].z = 0;
	}
}

void undistort_points (struct blob *blobs, int num_blobs,
		    vec3f *out_points,
		    double camera_matrix[9], double dist_coeffs[5], bool dist_fisheye)
{
	cv::Mat fishK = cv::Mat(3, 3, CV_64FC1, camera_matrix);
	cv::Mat fishDistCoeffs;
	std::vector<cv::Point2f> list_points2d(num_blobs);
	std::vector<cv::Point2f> list_points2d_undistorted(num_blobs);
  int i;

	for (i = 0; i < num_blobs; i++) {
		list_points2d[i].x = blobs[i].x;
		list_points2d[i].y = blobs[i].y;
  }

  /* Need to support 2 different distortion models. Fisheye for CV1, Pinhole for DK2 */
  if (dist_fisheye) {
    fishDistCoeffs = cv::Mat(4, 1, CV_64FC1, dist_coeffs);

		// we have distortion params for the openCV fisheye model
		// so we undistort the image points manually before passing them to the PnpRansac solver
		// and we give the solver identity camera + null distortion matrices
		cv::fisheye::undistortPoints(list_points2d, list_points2d_undistorted, fishK, fishDistCoeffs);
  }
  else {
    fishDistCoeffs = cv::Mat(5, 1, CV_64FC1, dist_coeffs);
		// Pinhole camera undistort
    cv::undistortPoints(list_points2d, list_points2d_undistorted, fishK, fishDistCoeffs);
  }

	for (i = 0; i < num_blobs; i++) {
		out_points[i].x = list_points2d_undistorted[i].x;
		out_points[i].y = list_points2d_undistorted[i].y;
		out_points[i].z = 1.0;
  }
}

/*
 * Refine pose operations on distortion correction UV image points,
 * with values near the range [-1,1] (a bit bigger depending on the
 * lens distortion). The image_points paramater is double[num_matches][2]
 */
extern "C" void refine_pose(vec3f *image_points,
		    rift_led **leds, int num_matches,
		    quatf *rot, vec3f *trans, double *reprojection_error)
{
  int i;
	cv::Mat dummyK = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat dummyD = cv::Mat::zeros(4, 1, CV_64FC1);
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);

	for (i = 0; i < 3; i++)
		tvec.at<double>(i) = trans->arr[i];

	quatf_to_3x3 (R, rot);
	cv::Rodrigues(R, rvec);

	std::vector<cv::Point3f> list_points3d(num_matches);
	std::vector<cv::Point2f> list_points2d(num_matches);

	for (i = 0; i < num_matches; i++) {
		list_points3d[i].x = leds[i]->pos.x;
		list_points3d[i].y = leds[i]->pos.y;
		list_points3d[i].z = leds[i]->pos.z;
		list_points2d[i].x = image_points[i].x;
		list_points2d[i].y = image_points[i].y;
	}

  // OpenCV 3.4.7 introduced a method to go straight to refining the pose with LM:
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 3 && CV_VERSION_MINOR > 4) || (CV_VERSION_MAJOR == 3 && CV_VERSION_MINOR == 4 && CV_VERSION_REVISION >= 7) 
  cv::solvePnPRefineLM (list_points3d, list_points2d, dummyK, dummyD, rvec, tvec);
#else
	cv::solvePnPRansac(list_points3d, list_points2d, dummyK, dummyD, rvec, tvec,
			   true, 10, 1.0 / 300.0, 0.95);
#endif

	vec3f v;
	double angle = sqrt(rvec.dot(rvec));
	double inorm = 1.0f / angle;

	v.x = rvec.at<double>(0) * inorm;
	v.y = rvec.at<double>(1) * inorm;
	v.z = rvec.at<double>(2) * inorm;
	oquatf_init_axis (rot, &v, angle);

	for (i = 0; i < 3; i++)
		trans->arr[i] = tvec.at<double>(i);

  /* Calculate reprojection error */
  if (reprojection_error) {
    double sq_error = 0.0;

    for (i = 0; i < num_matches; i++) {
      vec3f pos;
      double dx, dy;

      oquatf_get_rotated (rot, &leds[i]->pos, &pos);
      ovec3f_add (&pos, trans, &pos);
      ovec3f_multiply_scalar (&pos, 1.0/pos.z, &pos);

      dx = pos.x - image_points[i].x;
      dy = pos.y - image_points[i].y;

      sq_error += (dx*dx) + (dy*dy);
    }

    *reprojection_error = sq_error;
  }
}

