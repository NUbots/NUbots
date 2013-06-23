#ifndef BALLDETECTORDAVE_H
#define BALLDETECTORDAVE_H

#include <stdio.h>
#include <iostream>

//#include "Tools/Math/Line.h"
#include <list>
#include <vector>
#include "Vision/basicvisiontypes.h"
#include "Vision/Modules/balldetector.h"
#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/VisionTools/classificationcolours.h"
#include "Vision/VisionTypes/VisionFieldObjects/ball.h"

class BallDetectorDave : public BallDetector
{
public:
    BallDetectorDave();
    virtual ~BallDetectorDave();
    /*! @brief Detects a single ball from orange transitions using a geometric mean for general location
      and close classification at the pixel level combined with occlusion detection for refinement.
    */
    virtual std::vector<Ball> run();

protected:
    void appendEdgesFromSegments(const std::vector<ColourSegment>& segments, std::vector<Point>& pointList);
};

#endif // BALLDETECTION_H
