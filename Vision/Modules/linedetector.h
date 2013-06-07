#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H

#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/VisionTypes/greenhorizon.h"
#include "Vision/VisionTypes/nupoint.h"
#include "Vision/VisionTypes/VisionFieldObjects/fieldline.h"
#include <vector>

using std::vector;

class LineDetector
{
public:
    LineDetector();
    virtual ~LineDetector();

    virtual vector<FieldLine> run(const vector<NUPoint>& points) = 0;

protected:
    vector<pair<LSFittedLine, LSFittedLine> > mergeColinear(vector<pair<LSFittedLine, LSFittedLine> > lines,
                                                            double angle_threshold, double distance_threshold) const;

};

#endif // LINEDETECTOR_H
