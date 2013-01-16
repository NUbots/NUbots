#include "goaldetectorransac.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"
#include "Vision/GenericAlgorithms/ransac.h"

#include <limits>
#include <stdlib.h>
#include <boost/foreach.hpp>



GoalDetectorRANSAC::GoalDetectorRANSAC()
{
    m_n = 10;               //min pts to line essentially
    m_k = 40;               //number of iterations per fitting attempt
    m_e = 8.0;              //consensus margin
    m_max_iterations = 3;  //hard limit on number of fitting attempts
}

//for debugging only
#include "Infrastructure/NUImage/ColorModelConversions.h"

float test_e;
void updateE(int big_e, void * object) {
    test_e = big_e/20.0;
}

void GoalDetectorRANSAC::run()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();

    //get transitions associated with goals
    vector<ColourSegment> h_segments = vbb->getHorizontalTransitions(VisionFieldObject::GOAL_COLOUR);
    //vector<ColourSegment> v_segments = vbb->getVerticalTransitions(VisionFieldObject::GOAL_COLOUR);
    vector<Point> start_points, end_points;
    vector<LSFittedLine> start_lines, end_lines;
    vector<Quad> candidates;

    BOOST_FOREACH(ColourSegment s, h_segments) {
        start_points.push_back(Point(s.getStart().x, s.getStart().y));
        end_points.push_back(Point(s.getEnd().x, s.getEnd().y));
    }

    //use generic ransac implementation to find lines
    start_lines = RANSAC::findMultipleLines(start_points, m_e, m_n, m_k, m_max_iterations);
    end_lines = RANSAC::findMultipleLines(end_points, m_e, m_n, m_k, m_max_iterations);

    DataWrapper::getInstance()->debugPublish(DataWrapper::DBID_GOAL_LINES_START, start_lines);
    DataWrapper::getInstance()->debugPublish(DataWrapper::DBID_GOAL_LINES_END, end_lines);

    //MAKE QUADS FROM LINES
    candidates = buildGoalsFromLines(start_lines, end_lines);
}

vector<Quad> GoalDetectorRANSAC::buildGoalsFromLines(const vector<LSFittedLine> &start_lines, const vector<LSFittedLine> &end_lines)
{
    return vector<Quad>();
}

vector<Point> GoalDetectorRANSAC::getEdgePointsFromSegments(const vector<ColourSegment> &segments)
{
    vector<Point> points;
    BOOST_FOREACH(ColourSegment s, segments) {
        points.push_back(Point(s.getStart().x, s.getStart().y));
        points.push_back(Point(s.getEnd().x, s.getEnd().y));
    }

    return points;
}
