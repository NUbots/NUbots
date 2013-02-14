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
    virtual ~LineDetector();

    virtual vector<LSFittedLine> run(const vector<Point>& points) = 0;

protected:
    vector<LSFittedLine> mergeColinear(vector<LSFittedLine> lines, double angle_threshold, double distance_threshold) const;

};

#endif // LINEDETECTOR_H
