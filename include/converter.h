#ifndef MY_CONVERTER_H
#define MY_CONVERTER_H

#include "common_headers.h"
#include "rgbdframe.h"
#include "Thirdparty/orbslam_modified/include/Converter.h"

/* *
 * convert the common types in cv, eigen and g2o.
 */

namespace rgbd_tutor {
class Converter: public ORB_SLAM2::Converter
{
public:
    static  g2o::SE3Quat    toSE3Quat(const Eigen::Isometry3d& T)
    {
        Eigen::Matrix3d R = T.rotation();
        /*
        R << T(0,0) << T(0,1) << T(0,2) <<
             T(1,0) << T(1,1) << T(1,2) <<
          