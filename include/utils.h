#ifndef UTILS_H
#define UTILS_H

#include "common_headers.h"

namespace rgbd_tutor
{
	struct CAMERA_INTRINSIC_PARAMETERS
	{
	    // 标准内参
	    double cx=0, cy=0, fx=0, fy=0, scale=0;
	    // 畸变因子
	    double d0=0, d1=0, d2=0, d3=0, d4=0;
	};

	inline double norm_translate( const Eigen