#ifndef BALLDETECTION_H
#define BALLDETECTION_H

#include <stdio.h>
#include <iostream>

//#include "Tools/Math/Line.h"

#include "Vision/visionblackboard.h"
#include "Vision/VisionTools/classificationcolours.h"
#include "Vision/VisionTypes/quad.h"

using namespace std;

class BallDetector
{
public:
    BallDetector();
    virtual ~BallDetector();
    /*! @brief Detects a single ball from orange transitions using a geometric mean for general location
      and close classification at the pixel level combined with occlusion detection for refinement.
    */
    virtual vector<Ball> run();

protected:
    void appendEdgesFromSegments(const vector<ColourSegment>& segments, vector<Point>& pointlist);
};

#endif // BALLDETECTION_H
