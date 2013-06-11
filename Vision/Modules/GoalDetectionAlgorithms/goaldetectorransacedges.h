#ifndef GOALDETECTORRANSACEDGES_H
#define GOALDETECTORRANSACEDGES_H

#include "Vision/Modules/goaldetector.h"
#include "Tools/Math/LSFittedLine.h"
#include "Kinematics/Horizon.h"
#include <vector>
#include <list>

using std::vector;
using std::list;

class GoalDetectorRANSACEdges : public GoalDetector
{
public:
    GoalDetectorRANSACEdges();
    virtual std::vector<Goal> run();

private:
    //std::vector<Goal> assignGoals(const std::list<Quad>& post_candidates, const Quad& crossbar) const;
    std::list<Quad> buildQuadsFromLines(const std::vector<LSFittedLine>& start_lines,
                                   const std::vector<LSFittedLine>& end_lines,
                                   double tolerance);

    unsigned int getClosestUntriedLine(const LSFittedLine& start,
                                       const std::vector<LSFittedLine>& end_lines,
                                       std::vector<bool>& tried);

    std::vector<Goal> assignGoals(const std::list<Quad>& candidates, const Quad& crossbar) const;

    std::vector<Point> getEdgePointsFromSegments(const std::vector<ColourSegment> &segments);


    unsigned int m_n,
                 m_k,
                 m_max_iterations;
    float m_e;
};

#endif // GOALDETECTORRANSACEDGES_H
