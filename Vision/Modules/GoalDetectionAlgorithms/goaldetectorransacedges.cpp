#include "goaldetectorransacedges.h"
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

GoalDetectorRANSACEdges::GoalDetectorRANSACEdges()
{
    //m_n = 10;               //min pts to line essentially
    m_n = 30 / VisionConstants::HORIZONTAL_SCANLINE_SPACING;               //min pts to line essentially
    m_k = 40;               //number of iterations per fitting attempt
    m_e = 12.0;              //consensus margin
    m_max_iterations = 3;  //hard limit on number of fitting attempts
}

vector<Goal> GoalDetectorRANSACEdges::run()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    //get transitions associated with goals
    vector<ColourSegment> h_segments = vbb->getHorizontalTransitions(GOAL_COLOUR),
                          v_segments = vbb->getVerticalTransitions(GOAL_COLOUR);
    list<Quad> candidates;
    vector<Goal> posts;

    //finds the edge lines and constructs goals from that
    vector<Point> start_points, end_points;
    vector<pair<RANSACLine<Point>, vector<Point> > > ransac_results;
    vector<LSFittedLine> start_lines, end_lines;

    //get edge points
    BOOST_FOREACH(ColourSegment s, h_segments) {
        start_points.push_back(s.getStart());
        end_points.push_back(s.getEnd());
    }

    //use generic ransac implementation to find lines
    ransac_results = RANSAC::findMultipleModels<RANSACLine<Point>, Point>(start_points, m_e, m_n, m_k, m_max_iterations, RANSAC::BestFittingConsensus);
    for(size_t i=0; i<ransac_results.size(); i++) {
        start_lines.push_back(LSFittedLine(ransac_results.at(i).second));
    }

    ransac_results = RANSAC::findMultipleModels<RANSACLine<Point>, Point>(end_points, m_e, m_n, m_k, m_max_iterations, RANSAC::BestFittingConsensus);
    for(size_t i=0; i<ransac_results.size(); i++) {
        end_lines.push_back(LSFittedLine(ransac_results.at(i).second));
    }

//#if VISION_GOAL_VERBOSITY > 1
    DataWrapper::getInstance()->debugPublish(DBID_GOAL_LINES_START, start_lines);
    DataWrapper::getInstance()->debugPublish(DBID_GOAL_LINES_END, end_lines);
//#endif

    //Build candidates out of lines
    candidates = buildQuadsFromLines(start_lines, end_lines, VisionConstants::GOAL_RANSAC_MATCHING_TOLERANCE);

    cout << candidates.size() << " ";

    removeInvalid(candidates);

    cout << candidates.size() << " ";

    // OVERLAP CHECK
    mergeOverlapping(candidates);

    cout << candidates.size() << " ";

    posts = assignGoals(candidates);

    cout << posts.size() << endl;

    //Improves bottom centre estimate using vertical transitions
    BOOST_FOREACH(ColourSegment v, v_segments) {
        BOOST_FOREACH(Goal g, posts) {
            if(v.getEnd().y > g.getLocationPixels().y)
                g.setBase(v.getEnd());
        }
    }

#if VISION_GOAL_VERBOSITY > 0
    DataWrapper::getInstance()->debugPublish(DBID_GOALS_RANSAC_EDGES, posts);
#endif

    return posts;
}

list<Quad> GoalDetectorRANSACEdges::buildQuadsFromLines(const vector<LSFittedLine>& start_lines, const vector<LSFittedLine>& end_lines, double tolerance)
{
    // (must match exactly) 0 <= tolerance <= 1 (any pair will be accepted)
    //
    // LSFittedLine objects contain lists of points and can be quite large,
    // therefore it is more efficient to pass by const reference and maintain
    // a list of matched end lines than to pass by copy so that the end lines
    // list can be shrunk.

    if( tolerance < 0 or tolerance > 1)
        throw "GoalDetectorRANSACEdges::buildQuadsFromLines - tolerance must be in [0, 1]";

    list<Quad> quads;
    vector<bool> used(end_lines.size(), false);

#if VISION_GOAL_VERBOSITY > 2
    debug << "GoalDetectorRANSACEdges::buildQuadsFromLines - ";
#endif


    BOOST_FOREACH(const LSFittedLine& s, start_lines) {
        vector<bool> tried(used);   //consider all used lines tried
        vector<LSFittedLine>::const_iterator e_it;
        bool matched = false;

        //try end lines in order of closeness
        for(unsigned int i = getClosestUntriedLine(s, end_lines, tried);
            i < end_lines.size() && !matched;
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
                                matched = true;
//#if VISION_GOAL_VERBOSITY > 2
#if 1
                                debug << "\tsuccess " << sp1 << " " << sp2 << " , " << ep1 << " " << ep2 << endl;
                            }
                            else
                                debug << "\tline ordering fail: " << 0.5*(sp1.x + sp2.x) << " " << 0.5*(ep1.x + ep2.x) << endl;
                        }
                        else
                            debug << "\tnum points fail: " << n1 << " " << n2 << endl;
                    }
                    else
                        debug << "\tlength fail: " << l1 << " " << l2 << endl;
                }
                else
                    debug << "\tno endpoints" << endl;
            }
            else
                debug << "angle fail: " << s.getAngleBetween(e) << " > " << tolerance*mathGeneral::PI*0.5 << endl;
#else
                            }
                        }
                    }
                }
            }
#endif
        }
    }

    return quads;
}

unsigned int GoalDetectorRANSACEdges::getClosestUntriedLine(const LSFittedLine& start, const vector<LSFittedLine>& end_lines, vector<bool>& tried)
{
    if(end_lines.size() != tried.size())
        throw "GoalDetectorRANSACEdges::getClosestUntriedLine - 'end_lines' must match 'tried' in size";

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

vector<Point> GoalDetectorRANSACEdges::getEdgePointsFromSegments(const vector<ColourSegment> &segments)
{
    vector<Point> points;
    BOOST_FOREACH(ColourSegment s, segments) {
        points.push_back(Point(s.getStart().x, s.getStart().y));
        points.push_back(Point(s.getEnd().x, s.getEnd().y));
    }

    return points;
}
