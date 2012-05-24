#ifndef BALLDETECTION_H
#define BALLDETECTION_H

#include <stdio.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#include "Tools/Math/Line.h"

#include "Vision/visionblackboard.h"
#include "Vision/VisionTools/classificationcolours.h"
#include "Vision/VisionTypes/quad.h"

#include "Vision/visionconstants.h"

using namespace std;
using namespace cv;


class BallDetection
{
public:
    static void detectBall();
};

#endif // BALLDETECTION_H
