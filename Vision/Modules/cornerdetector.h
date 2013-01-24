#ifndef CORNERDETECTOR_H
#define CORNERDETECTOR_H

#include "Vision/VisionTypes/VisionFieldObjects/cornerpoint.h"

class CornerDetector
{
public:
    CornerDetector();

    vector<CornerPoint> run() const;

};

#endif // CORNERDETECTOR_H
