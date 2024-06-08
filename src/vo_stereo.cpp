/*
   IRTES-SET laboratory
   Authors: You Li (liyou026@gmail.com)
   Descirption: This a sample code of my PhD works
*/

#include "vo_stereo.hpp"

using namespace std;

VisualOdometryStereo::VisualOdometryStereo (parameters param) : param(param), VisualOdometry(param)
{
}

VisualOdometryStereo::~VisualOdometryStereo() {
}


bool VisualOdometryStereo::Process(QuadFeatureMatch& quadmatcher)
{

    quadmatches.clear();
    int num = quadmatcher.quadmatches.size();

    //copy to visual odometry
    for(int i = 0; i < num; i++)
    {
        pmatch tmp;
        tmp.u1c = quadmatcher.quadmatches[i].u1c;
        tmp.v1c = quadmatcher.quadmatches[i].v1c;

        tmp.u2c = quadmatcher.quadmatches[i].u2c;
        tmp.v2c = quadmatcher.quadmatches[i].v2c;

        tmp.u1p = quadmatcher.quadmatches[i].u1p;
        tmp.v1p = quadmatcher.quadmatches[i].v1p;

        tmp.u2p = quadmatcher.quadmatches[i].u2p;
        tmp.v2p = quadmatcher.quadmatches[i].v2p;

        quadmatches.push_back(tmp);
    }

    return updateMotion();
}

vector<double> VisualOdometrySt