/*************************************************************************
	> File Name: orb.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年02月29日 星期一 12时14分06秒
 ************************************************************************/

#include <iostream>
#include "common_headers.h"
#include "converter.h"
#include "orb.h"
using namespace std;

using namespace rgbd_tutor;

vector<cv::DMatch> OrbFeature::match( const RGBDFrame::Ptr& frame1, const RGBDFrame::Ptr& frame2 ) const
{
    vector< vector<cv::DMatch