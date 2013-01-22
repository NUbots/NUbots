#include "goaldetectorransac.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"
#include "Vision/GenericAlgorithms/ransac.h"
#include "Tools/Math/General.h"

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

#include <opencv2/highgui/highgui.hpp>
int PRECIS = 100;
double TOL;

void set(int v, void* p) {
    TOL = v/(double)PRECIS;
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

    //debugging {
    char c = 0;
    cv::namedWindow("blah");
    cv::createTrackbar("tolerance [0, 1]", "blah", 0, PRECIS, set);
    while(c != 27) {
        cv::Mat mat(vbb->getOriginalImage().getHeight(), vbb->getOriginalImage().getWidth(), CV_8U);

        //MAKE QUADS FROM LINES
        candidates = buildQuadsFromLines(start_lines, end_lines, TOL);

        BOOST_FOREACH(Quad q, candidates) {
            q.render(mat, cv::Scalar(255, 255, 0), true);
        }

        cv::imshow("blah", mat);

        c = cv::waitKey();
    }

    // } debugging

}

//vector<Quad> GoalDetectorRANSAC::buildQuadsFromLines(const vector<LSFittedLine> &start_lines, const vector<LSFittedLine> &end_lines)
//{
//    vector<LSFittedLine>::const_iterator s_it = start_lines.begin(),
//                                         e_it = end_lines.begin(),
//                                         e_temp;

//    while(s_it != start_lines.end() && e_it != end_lines.end()) {
//        //move through until end line is after start line
//        //get the end points of each line
//        Point sp1 = s_it->getLeftPoint(),
//              sp2 = s_it->getRightPoint(),
//              ep1 = e_it->getLeftPoint(),
//              ep2 = e_it->getRightPoint(),
//              intersection;

//        //sort the points
//        double d1 = 0.5*( (sp1-ep1).abs() + (sp2-ep2).abs() ),
//               d2 = 0.5*( (sp2-ep1).abs() + (sp1-ep2).abs() );

//        if(d1 > d2) {
//            //sp1 should be paired with ep2
//            Point c(ep1); ep1=ep2; ep2=c;   //swap ep1 and ep2
//        }

//        //check if terminal points of start are left of terminal points of end
//        //if(sp1.x < ep1.x && sp1.x < ep2.x && sp2.x < ep1.x && sp2.x < ep2.x) {

//        if(sp1.x < ep1.x && sp2.x < ep2.x) {
//            if(s_it->getIntersection(*e_it, intersection)) {
//                //check if intersection point is below end points, otherwise don't consider
//                if(intersection.y > sp1.y && intersection.y > ep1.y &&
//                   intersection.y > sp2.y && intersection.y > ep2.y) {
//                    //consider

//                }
//                else {
//                    e_it++;
//                }
//            }
//            else {
//                //perfectly parallel lines - consider
//            }
//        }
//        else {
//            e_it++;
//        }





//        s_it++;
//    }

//    return vector<Quad>();
//}

vector<Quad> GoalDetectorRANSAC::buildQuadsFromLines(const vector<LSFittedLine>& start_lines, const vector<LSFittedLine>& end_lines, double tolerance)
{
    // (must match exactly) 0 <= tolerance <= 1 (any pair will be accepted)
    //
    // LSFittedLine objects contain lists of points and can be quite large,
    // therefore it is more efficient to pass by const reference and maintain
    // a list of matched end lines than to pass by copy so that the end lines
    // list can be shrunk.

    if( tolerance < 0 or tolerance > 1)
        throw "GoalDetectorRANSAC::buildQuadsFromLines - tolerance must be in [0, 1]";

    vector<Quad> quads;
    vector<bool> used(end_lines.size(), false);

    BOOST_FOREACH(const LSFittedLine& s, start_lines) {
        vector<bool>::iterator u = used.begin();
        BOOST_FOREACH(const LSFittedLine& e, end_lines) {
            //check if the end line has already been used
            if(!(*u)) {
                //check angles
                if(s.getAngleBetween(e) < tolerance*mathGeneral::PI*0.5) { //dodgy way (linear with angle between)
                //if(min(a1/a2, a2/a1) <= (1-tolerance)) {
                    //get the end points of each line
                    Point sp1 = s.getLeftPoint(),
                          sp2 = s.getRightPoint(),
                          ep1 = e.getLeftPoint(),
                          ep2 = e.getRightPoint();
                    //find lengths of line segments
                    double l1 = (sp1 - sp2).abs(),
                           l2 = (ep1 - ep2).abs();
                    //check lengths
                    if(min(l1/l2, l2/l1) >= (1-tolerance)) {
                        //get num points
                        double n1 = s.getNumPoints(),
                               n2 = e.getNumPoints();
                        if(min(n1/n2, n2/n1) >= (1-tolerance)) {
                            //success
                            //order points
                            if(sp1.y < sp2.y) {
                                swap(sp1, sp2);
                            }
                            if(ep1.y < ep2.y) {
                                swap(ep1, ep2);
                            }
                            quads.push_back(Quad(sp1, sp2, ep2, ep1));  //generate candidate
                            *u = true;  //remove end line from consideration
                            cout << "success " << sp1 << sp2 << ep2 << ep1 << endl;
                            break;  //do not consider any more lines

                        }
                        else
                            cout << "num points fail: " << n1 << " " << n2 << endl;
                    }
                    else
                        cout << "length fail: " << l1 << " " << l2 << endl;
                }
                else
                    cout << "angle fail: " << s.getAngleBetween(e) << " > " << tolerance*mathGeneral::PI*0.5 << endl;
            }
            u++;
        }
    }

    return quads;
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
