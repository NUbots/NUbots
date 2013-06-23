#ifndef BALLDETECTOR_H
#define BALLDETECTOR_H

#include <stdio.h>
#include <iostream>

//#include "Tools/Math/Line.h"
#include <list>
#include <vector>
#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/VisionTools/classificationcolours.h"
#include "Vision/VisionTypes/VisionFieldObjects/ball.h"

class BallDetector
{
public:
    BallDetector();
    virtual ~BallDetector();
    /*! @brief Detects a single ball from orange transitions using a geometric mean for general location
      and close classification at the pixel level combined with occlusion detection for refinement.
    */
    virtual std::vector<Ball> run() = 0;
};

#endif // BALLDETECTION_H
