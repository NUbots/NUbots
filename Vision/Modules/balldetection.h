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

using namespace std;

class BallDetection
{
public:
    /*! @brief A static function to detect a single ball from orange transitions using a geometric mean for locating
      and close classification at the pixel level combined with occlusion detection for classifying.
    */
    static void detectBall();

private:
    static void appendEdgesFromSegments(const vector<ColourSegment>& segments, vector<Vector2<double> >& pointlist);
};

#endif // BALLDETECTION_H
