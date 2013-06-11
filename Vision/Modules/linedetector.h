#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H

#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/VisionTypes/greenhorizon.h"
#include "Vision/VisionTypes/groundpoint.h"
#include "Vision/VisionTypes/VisionFieldObjects/fieldline.h"
#include <vector>

using std::vector;

class LineDetector
{
public:
    LineDetector();
    virtual ~LineDetector();

    virtual std::vector<FieldLine> run(const std::vector<GroundPoint>& points) = 0;

protected:
    std::vector<std::pair<LSFittedLine, LSFittedLine> > mergeColinear(std::vector<std::pair<LSFittedLine, LSFittedLine> > lines,
                                                            double angle_threshold, double distance_threshold) const;

};

#endif // LINEDETECTOR_H
