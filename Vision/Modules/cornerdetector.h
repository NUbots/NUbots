#ifndef CORNERDETECTOR_H
#define CORNERDETECTOR_H

#include "Vision/VisionTypes/VisionFieldObjects/cornerpoint.h"
#include "Vision/VisionTypes/VisionFieldObjects/fieldline.h"

class CornerDetector
{
public:
    CornerDetector(double tolerance);

    vector<CornerPoint> run(const vector<FieldLine>& lines) const;

    void setTolerance(double tolerance);

private:
    CornerPoint::TYPE findCorner(Vector2<GroundPoint> ep1, Vector2<GroundPoint> ep2, GroundPoint intersection, double tolerance) const;

    double m_tolerance;
};

#endif // CORNERDETECTOR_H
