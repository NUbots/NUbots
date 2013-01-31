#ifndef CORNERDETECTOR_H
#define CORNERDETECTOR_H

#include "Vision/VisionTypes/VisionFieldObjects/cornerpoint.h"

class CornerDetector
{
public:
    CornerDetector();

    vector<CornerPoint> run() const;

private:
    CornerPoint::TYPE findCorner(Vector2<Point> ep1, Vector2<Point> ep2, Point intersection, double tolerance) const;
};

#endif // CORNERDETECTOR_H
