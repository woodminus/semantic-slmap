#ifndef FRAME_H
#define FRAME_H

#include "segnet.h"
#include "common_headers.h"
#include "parameter_reader.h"
#include "feature.h"
#include "utils.h"

#include"Thirdparty/DBoW2/DBoW2/FORB.h"
#include"Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <mutex>

/**
 * RGBDFrame 
 * 该类记录了每个帧的信息。帧是slam的基本单位。
 * 本身可以看作一个struct。
 * 由于Pose可能被若干线程同时访问，需要加锁。
 */

namespace rgbd_tutor{

class RGBDFrame //帧
{
public:
    RGBDFrame()
    { 
        color = cv::imread("/home/relaybot/mumu/segnet-slam/models/color.png", 1);
    }

    typedef shared_ptr<RGBDFrame> Ptr;//智能指针  命名空间std  C++标准库

    // 数据成员
    int id = -1;                //-1表示该帧不存在
    cv::Mat rgb, depth, disparity, semantic, raw_semantic, color;
    cv::Mat result;
    cv::Mat img_l