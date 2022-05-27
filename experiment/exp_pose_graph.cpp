#include "track.h"
#include "pose_graph.h"
#include "vo_stereo.hpp"
#include "readGTPose.h"

using namespace rgbd_tutor;

int main()
{
    ParameterReader	parameterReader;

    VisualOdometryStereo::parameters voparam; 
    double f = parameterReader.getData<doubl