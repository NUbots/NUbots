#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H

#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/VisionTypes/greenhorizon.h"
#include "Tools/Math/LSFittedLine.h"
#include <vector>

using std::vector;

class LineDetector
{
public:
    LineDetector();

    virtual void run() = 0;

protected:
    vector<LSFittedLine> mergeColinear(vector<LSFittedLine> lines, double angle_threshold, double distance_threshold) const;
    vector<Point> getPointsFromSegments(const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments) const;
    vector<Point> pointsUnderGreenHorizon(const vector<Point>& points, const GreenHorizon& gh) const;

};

#endif // LINEDETECTOR_H
