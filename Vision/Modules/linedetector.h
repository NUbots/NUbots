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
    enum METHOD {
        SAM,
        RANSAC
    };

    LineDetector();
    LineDetector(METHOD method);

    void run();

private:
    SplitAndMerge m_SAM;
    METHOD m_method;

    vector<LinePoint*> getPointsFromSegments(const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments);
    vector<LinePoint*> pointsUnderGreenHorizon(const vector<LinePoint*> points, const GreenHorizon& gh);

};

#endif // LINEDETECTOR_H
