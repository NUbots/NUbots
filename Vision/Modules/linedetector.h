#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H

#include "Vision/Modules/LineDetectionAlgorithms/splitandmerge.h"
#include "Vision/VisionTypes/transition.h"
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

    vector<LinePoint*> getPointsFromTransitions(const vector<Transition>& h_trans, const vector<Transition>& v_trans);
    vector<LinePoint*> pointsUnderGreenHorizon(const vector<LinePoint*> points, const GreenHorizon& gh);

};

#endif // LINEDETECTOR_H
