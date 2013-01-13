#ifndef GOALDETECTORRANSAC_H
#define GOALDETECTORRANSAC_H

#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/Modules/goaldetector.h"
#include "Tools/Math/LSFittedLine.h"
#include <vector>

using std::vector;

class GoalDetectorRANSAC : public GoalDetector
{
public:
    GoalDetectorRANSAC();
    void run();

private:
    vector<LinePoint> getEdgePointsFromSegments(const vector<ColourSegment> &segments);

/*    unsigned*/ int m_n,
                 m_k,
                 m_max_iterations;
    float m_e;
};

#endif // GOALDETECTORRANSAC_H
