#ifndef GOALDETECTORRANSAC_H
#define GOALDETECTORRANSAC_H

#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/VisionTypes/quad.h"
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
    vector<Point> getEdgePointsFromSegments(const vector<ColourSegment> &segments);

    vector<Quad> buildQuadsFromLines(const vector<LSFittedLine>& start_lines, const vector<LSFittedLine>& end_lines);

/*    unsigned*/ int m_n,
                 m_k,
                 m_max_iterations;
    float m_e;
};

#endif // GOALDETECTORRANSAC_H
