#include "goaldetectorransac.h"
#include "debugverbosityvision.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"
#include "Vision/GenericAlgorithms/ransac.h"
#include "Vision/VisionTypes//RANSACTypes/ransacline.h"
#include "Vision/VisionTypes//RANSACTypes/ransacgoal.h"
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

//vector<Goal> GoalDetectorRANSAC::run()
//{
//    VisionBlackboard* vbb = VisionBlackboard::getInstance();
//    //get transitions associated with goals
//    vector<ColourSegment> h_segments = vbb->getHorizontalTransitions(GOAL_COLOUR),
//                          v_segments = vbb->getVerticalTransitions(GOAL_COLOUR);
//    vector<Quad> candidates;
//    vector<Goal> posts;

//    //finds the edge lines and constructs goals from that
//    vector<Point> start_points, end_points;
//    vector<pair<VisionLine, vector<Point> > > ransac_results;
//    vector<LSFittedLine> start_lines, end_lines;



//    //get edge points
//    BOOST_FOREACH(ColourSegment s, h_segments) {
//        start_points.push_back(s.getStart());
//        end_points.push_back(s.getEnd());
//    }

//    //use generic ransac implementation to find lines
//    ransac_results = RANSAC::findMultipleModels<VisionLine, Point>(start_points, m_e, m_n, m_k, m_max_iterations);
//    for(unsigned int i=0; i<ransac_results.size(); i++) {
//        start_lines.push_back(LSFittedLine(ransac_results.at(i).second));
//    }

//    ransac_results = RANSAC::findMultipleModels<VisionLine, Point>(end_points, m_e, m_n, m_k, m_max_iterations);
//    for(unsigned int i=0; i<ransac_results.size(); i++) {
//        end_lines.push_back(LSFittedLine(ransac_results.at(i).second));
//    }

//    DataWrapper::getInstance()->debugPublish(DBID_GOAL_LINES_START, start_lines);
//    DataWrapper::getInstance()->debugPublish(DBID_GOAL_LINES_END, end_lines);

//    //Build candidates out of lines
//    candidates = buildQuadsFromLines(start_lines, end_lines, VisionConstants::GOAL_RANSAC_MATCHING_TOLERANCE);

//    cout << "candidates: " << candidates.size() << endl;

//    //validity check
//    removeInvalidPosts(candidates);
//    cout << "after invalid: " << candidates.size() << endl;

//    //overlap check
//    overlapCheck(candidates);
//    cout << "after overlap: " << candidates.size() << endl;

//    posts = assignGoals(candidates);

//    //Improves bottom centre estimate using vertical transitions
//    BOOST_FOREACH(ColourSegment v, v_segments) {
//        BOOST_FOREACH(Goal g, posts) {
//            if(v.getEnd().y > g.getLocationPixels().y)
//                g.setBase(v.getEnd());
//        }
//    }

//    return posts;
//}

vector<Goal> GoalDetectorRANSAC::run()
{
    const float STDDEV_THRESHOLD = 1.5;
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    //get transitions associated with goals
    vector<ColourSegment> h_segments = vbb->getHorizontalTransitions(GOAL_COLOUR),
                          v_segments = vbb->getVerticalTransitions(GOAL_COLOUR);
    vector<Quad> candidates;
    vector<Goal> posts;

    //finds the centre lines and constructs goals from them
    vector<pair<RANSACGoal, vector<ColourSegment> > > ransac_results;
    vector<RANSACGoal> goal_lines;
    vector<pair<RANSACGoal, vector<ColourSegment> > >::iterator rit;

    Vector2<double> h_length_stats = calculateSegmentLengthStatistics(h_segments);

    // fill histogram bins
    vector<ColourSegment>::iterator it = h_segments.begin();
    while(it != h_segments.end()) {
        //use stddev throwout to remove topbar segments
        if(it->getLength() > h_length_stats.x + STDDEV_THRESHOLD*h_length_stats.y)
            it = h_segments.erase(it);
        else
            it++;
    }

    //use generic ransac implementation to find lines
    ransac_results = RANSAC::findMultipleModels<RANSACGoal, ColourSegment>(h_segments, m_e, m_n, m_k, m_max_iterations, RANSAC::BestFittingConsensus);

    for(rit = ransac_results.begin(); rit != ransac_results.end(); rit++) {
        rit->first.fit(rit->second);
        goal_lines.push_back(rit->first);
    }

    //cout << "lines: " << goal_lines.size() << endl;

    /// @todo MERGE COLINEAR / INTERSECTING

    //cout << "after merge: " << goal_lines.size() << endl;

#if VISION_FIELDOBJECT_VERBOSITY > 1
    vector<LSFittedLine> debug_lines;
    BOOST_FOREACH(const RANSACGoal& g, goal_lines) {
        debug_lines.push_back(g);
    }
    DataWrapper::getInstance()->debugPublish(DBID_GOAL_LINES_CENTRE, debug_lines);
#endif

    //Build candidates out of lines
    if(goal_lines.size() > 2) {
        //pick best two
        double min_angle_diff = std::numeric_limits<double>::max();
        pair<unsigned int, unsigned int> best(0,0);
        for(unsigned int i = 0; i < goal_lines.size(); i++) {
            for(unsigned int k = i+1; k < goal_lines.size(); k++) {
                double a = goal_lines[i].getAngleBetween(goal_lines[k]);
                if(a < min_angle_diff) {
                    min_angle_diff = a;
                    best.first = i;
                    best.second = k;
                }
            }
        }

        //add these to end and erase all but
        goal_lines.push_back(goal_lines[best.first]);
        goal_lines.push_back(goal_lines[best.second]);
        goal_lines.erase(goal_lines.begin(), goal_lines.end() - 2);
    }

    //cout << "after selection (if more than 2): " << goal_lines.size() << endl;

    BOOST_FOREACH(const RANSACGoal g, goal_lines) {
        Point ep1, ep2;
        if(g.getEndPoints(ep1, ep2)) {
            //get width vector (to right)
            Point w = (ep1 - ep2).normalize(g.getWidth()).rotateLeft()*0.5;

            if(ep1.y < ep2.y) {
                // pt0 is top
                candidates.push_back(Quad(ep2 - w, ep1 - w, ep1 + w, ep2 + w));
            }
            else {
                candidates.push_back(Quad(ep1 - w, ep2 - w, ep2 + w, ep1 + w));
            }
        }
        else {
            errorlog << "GoalDetectorRANSAC::run() - invalid RANSACGoal - no endpoints" << endl;
        }
    }

    //cout << "candidates: " << candidates.size() << endl;

    //validity check
    removeInvalidPosts(candidates);
    //cout << "after invalid removal: " << candidates.size() << endl;

    //overlap check
    overlapCheck(candidates);
    //cout << "after overlap removal: " << candidates.size() << endl;

    posts = assignGoals(candidates);

    //Improves bottom centre estimate using vertical transitions
    BOOST_FOREACH(ColourSegment v, v_segments) {
        BOOST_FOREACH(Goal g, posts) {
            if(v.getEnd().y > g.getLocationPixels().y)
                g.setBase(v.getEnd());
        }
    }

    return posts;
}



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
        vector<bool> tried(used);   //consider all used lines tried
        vector<LSFittedLine>::const_iterator e_it;

        //try end lines in order of closeness
        for(unsigned int i = getClosestUntriedLine(s, end_lines, tried);
            i < end_lines.size();
            i = getClosestUntriedLine(s, end_lines, tried)) {

            const LSFittedLine& e = end_lines.at(i);

            //check angles
            if(s.getAngleBetween(e) < tolerance*mathGeneral::PI*0.5) { //dodgy way (linear with angle between)
            //if(min(a1/a2, a2/a1) <= (1-tolerance)) {
                //get the end points of each line
                Point sp1, sp2,
                      ep1, ep2;
                if(s.getEndPoints(sp1, sp2) && e.getEndPoints(ep1, ep2)) {
                    //find lengths of line segments
                    double l1 = (sp1 - sp2).abs(),
                           l2 = (ep1 - ep2).abs();
                    //check lengths
                    if(min(l1/l2, l2/l1) >= (1-tolerance)) {
                        //get num points
                        double n1 = s.getNumPoints(),
                               n2 = e.getNumPoints();
                        if(min(n1/n2, n2/n1) >= (1-tolerance)) {
                            //check start is to left of end
                            if(0.5*(sp1.x + sp2.x) < 0.5*(ep1.x + ep2.x)) {
                                //success
                                //order points
                                if(sp1.y < sp2.y) {
                                    Point c = sp1; sp1 = sp2; sp2 = c;
                                }
                                if(ep1.y < ep2.y) {
                                    Point c = ep1; ep1 = ep2; ep2 = c;
                                }
                                quads.push_back(Quad(sp1, sp2, ep2, ep1));  //generate candidate
                                used.at(i) = true;  //remove end line from consideration
                                cout << "success " << sp1 << " " << sp2 << " , " << ep1 << " " << ep2 << endl;
                                break;  //do not consider any more lines
                            }
                            else
                                cout << "line ordering fail: " << 0.5*(sp1.x + sp2.x) << " " << 0.5*(ep1.x + ep2.x) << endl;
                        }
                        else
                            cout << "num points fail: " << n1 << " " << n2 << endl;
                    }
                    else
                        cout << "length fail: " << l1 << " " << l2 << endl;
                }
                else
                    cout << "no endpoints" << endl;
            }
            else
                cout << "angle fail: " << s.getAngleBetween(e) << " > " << tolerance*mathGeneral::PI*0.5 << endl;
        }
    }

    return quads;
}

unsigned int GoalDetectorRANSAC::getClosestUntriedLine(const LSFittedLine& start, const vector<LSFittedLine>& end_lines, vector<bool>& tried)
{
    if(end_lines.size() != tried.size())
        throw "GoalDetectorRANSAC::getClosestUntriedLine - 'end_lines' must match 'tried' in size";

    unsigned int best = end_lines.size();   //for if all have been tried
    double d_best = std::numeric_limits<double>::max();

    for(unsigned int i = 0; i < end_lines.size(); i++) {
        //check if tried yet
        if(!tried[i]) {
            //check if distance is smallest
            double d = start.averageDistanceBetween(end_lines[i]);
            if(d < d_best) {
                best = i;
                d_best = d;
            }
        }
    }

    if(best < end_lines.size())
        tried[best] = true; //mark as used

    return best;
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
