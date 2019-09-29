#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

extern "C" {
#include "kalman.h"
}

struct kalman_pose_s {
  int nMeasurements;

  double lastTime;
  cv::KalmanFilter *KF;
  vec3f pos_estd;
  vec3f rot_estd;
};

kalman_pose *kalman_pose_new(ohmd_context* ohmd_ctx)
{
  cv::KalmanFilter *KF;
  const int nStates = 18;
  const int nMeasurements = 6;
  const int nInputs = 0;

  kalman_pose *kp = (kalman_pose *) ohmd_alloc(ohmd_ctx, sizeof (kalman_pose));

  kp->lastTime = -1.0;
  kp->nMeasurements = 6;

  KF = kp->KF = new cv::KalmanFilter;
  kp->KF->init(nStates, nMeasurements, nInputs, CV_64F);              // init Kalman Filter

  /* Arbitrary starting noise values. FIXME */
  cv::setIdentity(KF->processNoiseCov, cv::Scalar::all(1e5));     // set process noise
  cv::setIdentity(KF->measurementNoiseCov, cv::Scalar::all(1e3)); // set measurement noise
  cv::setIdentity(KF->errorCovPost, cv::Scalar::all(1));          // error covariance

  /* MEASUREMENT MODEL */
  //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]

  KF->measurementMatrix.at<double>(0,0) = 1;  // x
  KF->measurementMatrix.at<double>(1,1) = 1;  // y
  KF->measurementMatrix.at<double>(2,2) = 1;  // z
  KF->measurementMatrix.at<double>(3,9) = 1;  // roll
  KF->measurementMatrix.at<double>(4,10) = 1; // pitch
  KF->measurementMatrix.at<double>(5,11) = 1; // yaw

  return kp;
}

static void
updateProcessModel (cv::KalmanFilter &KF, double dt)
{
  double accel = 0.5*dt*dt;
  // position
  KF.transitionMatrix.at<double>(0,3) = dt;
  KF.transitionMatrix.at<double>(1,4) = dt;
  KF.transitionMatrix.at<double>(2,5) = dt;
  KF.transitionMatrix.at<double>(3,6) = dt;
  KF.transitionMatrix.at<double>(4,7) = dt;
  KF.transitionMatrix.at<double>(5,8) = dt;
  KF.transitionMatrix.at<double>(0,6) = accel;
  KF.transitionMatrix.at<double>(1,7) = accel;
  KF.transitionMatrix.at<double>(2,8) = accel;
  // orientation
  KF.transitionMatrix.at<double>(9,12) = dt;
  KF.transitionMatrix.at<double>(10,13) = dt;
  KF.transitionMatrix.at<double>(11,14) = dt;
  KF.transitionMatrix.at<double>(12,15) = dt;
  KF.transitionMatrix.at<double>(13,16) = dt;
  KF.transitionMatrix.at<double>(14,17) = dt;
  KF.transitionMatrix.at<double>(9,15) = accel;
  KF.transitionMatrix.at<double>(10,16) = accel;
  KF.transitionMatrix.at<double>(11,17) = accel;
}

void kalman_pose_update(kalman_pose *kp, double time, const vec3f* pos, quatf *orient)
{
    if (kp->lastTime < 0)
        kp->lastTime = time;

    double dt = time - kp->lastTime;

    // Get the measured translation
    cv::Mat translation_measured(3, 1, CV_64F);
    cv::Mat rotation_measured(3, 3, CV_64F);
    vec3f eulers;
    cv::Mat measurements(kp->nMeasurements, 1, CV_64FC1);
    
    measurements.setTo(cv::Scalar(0));

    oquatf_get_euler_angles (orient, &eulers);

    // Set measurement to predict
    measurements.at<double>(0) = pos->x;
    measurements.at<double>(1) = pos->y;
    measurements.at<double>(2) = pos->z;
    measurements.at<double>(3) = eulers.arr[0]; // yaw
    measurements.at<double>(4) = eulers.arr[1]; // pitch
    measurements.at<double>(5) = eulers.arr[2]; // roll

    updateProcessModel(*kp->KF, dt);

    // First predict, to update the internal statePre variable
    cv::Mat prediction = kp->KF->predict();
    // The "correct" phase that is going to use the predicted value and our measurement
    cv::Mat estimated = kp->KF->correct(measurements);

    // Estimated translation
    kp->pos_estd.x = estimated.at<double>(0);
    kp->pos_estd.y = estimated.at<double>(1);
    kp->pos_estd.z = estimated.at<double>(2);

    kp->rot_estd.x = estimated.at<double>(9);
    kp->rot_estd.y = estimated.at<double>(10);
    kp->rot_estd.z = estimated.at<double>(11);
}

void kalman_pose_get_estimated(kalman_pose *kp, vec3f* pos, quatf *orient)
{
    *pos = kp->pos_estd;
    oquatf_from_euler_angles(orient, &kp->rot_estd);
}

void kalman_pose_free (kalman_pose *kp)
{
   delete kp->KF;
   free (kp);
}

