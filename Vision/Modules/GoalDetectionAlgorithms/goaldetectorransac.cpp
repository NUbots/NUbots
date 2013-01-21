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
    //candidates = buildQuadsFromLines(start_lines, end_lines);
}

vector<Quad> GoalDetectorRANSAC::buildQuadsFromLines(const vector<LSFittedLine> &start_lines, const vector<LSFittedLine> &end_lines)
{
    vector<LSFittedLine>::const_iterator s_it = start_lines.begin(),
                                         e_it = end_lines.begin(),
                                         e_temp;

    while(s_it != start_lines.end() && e_it != end_lines.end()) {
        //move through until end line is after start line
        Point sp1 = s_it->getLeftPoint(),
              sp2 = s_it->getRightPoint(),
              ep1 = e_it->getLeftPoint(),
              ep2 = e_it->getRightPoint(),
              intersection;

        //sort the points
        double d1 = 0.5*( (sp1-ep1).abs() + (sp2-ep2).abs() ),
               d2 = 0.5*( (sp2-ep1).abs() + (sp1-ep2).abs() );

        if(d1 > d2) {
            //sp1 should be paired with ep2

        }


        //check if terminal points of start are left of terminal points of end
        if(sp1.x < ep1.x && sp1.x < ep2.x && sp2.x < ep1.x && sp2.x < ep2.x) {
            if(s_it->getIntersection(*e_it, intersection)) {
                //check if intersection point is below end points, otherwise don't consider

            }
            else {
                //parallel lines
            }
        }
        else {
            e_it++;
        }





        s_it++;
    }

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
