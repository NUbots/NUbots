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
    vector<ColourSegment> v_segments = vbb->getVerticalTransitions(VisionFieldObject::GOAL_COLOUR),
                          h_segments = vbb->getHorizontalTransitions(VisionFieldObject::GOAL_COLOUR);
    vector<LinePoint> start_points, end_points;
    vector<LSFittedLine> start_lines, end_lines;
    vector<LSFittedLine>::iterator l_it;

    BOOST_FOREACH(ColourSegment s, h_segments) {
        start_points.push_back(LinePoint(s.getStart().x, s.getStart().y));
        end_points.push_back(LinePoint(s.getEnd().x, s.getEnd().y));
    }

    //use generic ransac implementation to fine lines
//        lines = RANSAC::findMultipleLines(h_points, m_e, m_n, m_k, m_max_iterations);

    //h_points.insert(h_points.end(), v_points.begin(), v_points.end());
    start_lines = RANSAC::findMultipleLines(start_points, m_e, m_n, m_k, m_max_iterations);
    end_lines = RANSAC::findMultipleLines(end_points, m_e, m_n, m_k, m_max_iterations);

    //debugging
    vector<FieldLine> s_l, e_l;
    BOOST_FOREACH(LSFittedLine l, start_lines) {
        s_l.push_back(FieldLine(l));
    }
    BOOST_FOREACH(LSFittedLine l, end_lines) {
        e_l.push_back(FieldLine(l));
    }

    DataWrapper::getInstance()->debugPublish(DataWrapper::DBID_GOAL_LINES_START, s_l);
    DataWrapper::getInstance()->debugPublish(DataWrapper::DBID_GOAL_LINES_END, e_l);

    //MAKE GOALS FROM LINES

}

vector<LinePoint> GoalDetectorRANSAC::getEdgePointsFromSegments(const vector<ColourSegment> &segments)
{
    vector<LinePoint> points;
    BOOST_FOREACH(ColourSegment s, segments) {
        points.push_back(LinePoint(s.getStart().x, s.getStart().y));
        points.push_back(LinePoint(s.getEnd().x, s.getEnd().y));
    }

    return points;
}
