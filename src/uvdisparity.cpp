/*
   IRTES-SET laboratory
   Authors: You Li (liyou026@gmail.com)
   Descirption: This a sample code of my PhD works
*/

#include "uvdisparity.hpp"
#include "stereo.h"
using namespace cv;
using namespace std;








USegmentPars::USegmentPars(int min_intense_, int min_disparity_raw_, int min_area_)
{
    min_intense = min_intense_;
    min_disparity_raw = min_disparity_raw_;
    min_area = min_area_;
}

//constructer
 UVDisparity::UVDisparity()
 {
     out_th_ = 6.0f;                    //outlier rejection threshold
     inlier_tolerance_ = 3;             //inlier tolerance threshold
     min_adjust_intense_ = 19;          //minum adjust intensity


     //init Kalman filter parameters
     pitch1_KF = new KalmanFilter(2,1,0);
     pitch1_KF->transitionMatrix = *(Mat_<float>(2, 2) << 1,0,0,1);
     cv::setIdentity(pitch1_KF->measurementMatrix);
     cv::setIdentity(pitch1_KF->processNoiseCov, Scalar::all(0.000005));
     cv::setIdentity(pitch1_KF->measurementNoiseCov, Scalar::all(0.001));
     cv::setIdentity(pitch1_KF->errorCovPost, Scalar::all(1));

     pitch2_KF = new KalmanFilter(2,1,0);
     pitch2_KF->transitionMatrix = *(Mat_<float>(2, 2) << 1,0,0,1);
     cv::setIdentity(pitch2_KF->measurementMatrix);
     cv::setIdentity(pitch2_KF->proc