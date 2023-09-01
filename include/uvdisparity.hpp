/*
   IRTES-SET laboratory
   Authors: You Li (liyou026@gmail.com)
   Descirption: This a sample code of my PhD works
*/

#ifndef UVDisparity__HPP
#define UVDisparity__HPP

#include <iostream>
#include "stereo.h"
#include "vo_stereo.hpp"

using namespace std;

//parameters in segmentation of U-disparity image
struct USegmentPars
{
  USegmentPars():min_intense(32), min_disparity_raw(64), min_area(40){};

  USegmentPars(int min_intense_, int min_disparity_raw_, int min_area_);

  inline USegmentPars& operator=(const USegmentPars& t)
    {
      min_intense = t.min_intense;
      min_disparity_raw = t.min_disparity_raw;
      min_area = t.min_area;
      return *this;
    }

  int min_intense;       // minum gray intensity for starting segmentation
  int min_disparity_raw; //minmm raw disparity for starting segmentation
  int min_area;          //minum area required for candidate mask
};


// intergrate a bundle of methods in U-V disparity image understanding
class UVDisparity
{
  public:
    //constructor and deconstructor
    UVDisparity();
    ~UVDisparity();

    // initialization functions
    inline void SetCalibPars(CalibPars& calib_par)
    {
        calib_ = calib_par;
    }
    inline void SetROI3D(ROI3D& roi_3d)
    {
        roi_ = roi_3d;
    }
    inline void SetUSegmentPars(int min_intense, int min_disparity_raw, int min_area)
    {
       