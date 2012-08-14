#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H

#include "Vision/Modules/LineDetectionAlgorithms/splitandmerge.h"
#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/VisionTypes/greenhorizon.h"
#include <vector>

using std::vector;

class LineDetector
{
public:
    LineDetector();

    void run();

private:
    SplitAndMerge m_SAM;

    vector<LinePoint*> getPointsFromSegments(const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments);
    vector<LinePoint*> pointsUnderGreenHorizon(const vector<LinePoint*> points, const GreenHorizon& gh);

};

#endif // LINEDETECTOR_H
