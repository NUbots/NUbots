#ifndef GOALDETECTORRANSACEDGES_H
#define GOALDETECTORRANSACEDGES_H

#include "Vision/Modules/goaldetector.h"
#include "Tools/Math/LSFittedLine.h"
#include <vector>

using std::vector;

class GoalDetectorRANSACEdges : public GoalDetector
{
public:
    GoalDetectorRANSACEdges();
    virtual vector<Goal> run();

private:
    vector<Quad> buildQuadsFromLines(const vector<LSFittedLine>& start_lines,
                                     const vector<LSFittedLine>& end_lines,
                                     double tolerance);

    unsigned int getClosestUntriedLine(const LSFittedLine& start,
                                       const vector<LSFittedLine>& end_lines,
                                       vector<bool>& tried);

    vector<Point> getEdgePointsFromSegments(const vector<ColourSegment> &segments);


    unsigned int m_n,
                 m_k,
                 m_max_iterations;
    float m_e;
};

#endif // GOALDETECTORRANSACEDGES_H
