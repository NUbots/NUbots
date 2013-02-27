#include "goaldetectorransaccentres.h"
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

GoalDetectorRANSACCentres::GoalDetectorRANSACCentres()
{
    //m_n = 10;               //min pts to line essentially
    m_n = 30 / VisionConstants::HORIZONTAL_SCANLINE_SPACING;               //min pts to line essentially
    m_k = 40;               //number of iterations per fitting attempt
    m_e = 12.0;              //consensus margin
    m_max_iterations = 5;  //hard limit on number of fitting attempts
}

vector<Goal> GoalDetectorRANSACCentres::run()
{
    const double ANGLE_TOLERANCE = 0.15;
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    const Horizon& kh = vbb->getKinematicsHorizon();
    //get transitions associated with goals
    vector<ColourSegment> h_segments = vbb->getHorizontalTransitions(GOAL_COLOUR),
                          v_segments = vbb->getVerticalTransitions(GOAL_COLOUR);
    list<Quad> candidates;
    vector<Goal> posts;

    //finds the centre lines and constructs goals from them
    vector<pair<RANSACGoal, vector<ColourSegment> > > ransac_results;
    vector<pair<RANSACGoal, vector<ColourSegment> > >::iterator rit;
    vector<RANSACGoal> goal_lines;

//    Vector2<double> h_length_stats = calculateSegmentLengthStatistics(h_segments);

//    //use stddev throwout to remove topbar segments
//    vector<ColourSegment>::iterator it = h_segments.begin();
//    while(it != h_segments.end()) {
//        if(it->getLength() > h_length_stats.x + STDDEV_THRESHOLD*h_length_stats.y)
//            it = h_segments.erase(it);
//        else
//            it++;
//    }

    //use generic ransac implementation to find lines
    ransac_results = RANSAC::findMultipleModels<RANSACGoal, ColourSegment>(h_segments, m_e, m_n, m_k, m_max_iterations, RANSAC::BestFittingConsensus);

    for(rit = ransac_results.begin(); rit != ransac_results.end(); rit++) {
        rit->first.fit(rit->second);
        goal_lines.push_back(rit->first);
    }

//    cout << "lines: " << goal_lines.size() << endl;

    /// @todo MERGE COLINEAR

//    cout << "after merge: " << goal_lines.size() << endl;

#if VISION_GOAL_VERBOSITY > 1
    vector<RANSACGoal> debug_lines;
    BOOST_FOREACH(const RANSACGoal& g, goal_lines) {
        debug_lines.push_back(g);
    }
    DataWrapper::getInstance()->debugPublish(DBID_GOAL_LINES_CENTRE, goal_lines);
#endif

    //Build candidates out of lines
    if(goal_lines.size() > 2) {
        //pick best two - i.e. most vertical
        pair<double, double> largest_angles = pair<double, double>(-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max());
        pair<size_t, size_t> best(0,0);
        for(size_t i = 0; i < goal_lines.size(); i++) {
            double a = kh.getAngleBetween(goal_lines[i].getLine());
            if(a > largest_angles.first) {
                largest_angles.first = a;
                best.first = i;
            }
            else if(a > largest_angles.second) {
                largest_angles.second = a;
                best.second = i;
            }
        }

        //add these to end and erase all but
        RANSACGoal g1 = goal_lines[best.first],
                   g2 = goal_lines[best.second];

        cout << largest_angles.first << " " << largest_angles.second << endl;

        goal_lines.clear();
        goal_lines.push_back(g1);
        goal_lines.push_back(g2);
    }


    //cout << "after selection (if more than 2): " << goal_lines.size() << endl;

    BOOST_FOREACH(const RANSACGoal g, goal_lines) {
        //don't add goals that aren't vertical enough
        if(kh.getAngleBetween(g.getLine()) >= mathGeneral::PI*0.5*(1-ANGLE_TOLERANCE)) {
            pair<Point, Point> endpts = g.getEndPoints();
            pair<double, double> widths = g.getWidths();
            Vector2<double> dir = g.getDirection();

            // compensate width for angle between goal and segments
            double sin_theta = 1;

            if( dir.x != 0 && dir.y != 0 )
                sin_theta = dir.y / dir.abs();

            //get width vector (to right)
            Point w1 = dir.normalize(sin_theta*widths.first*0.5).rotateRight(),
                  w2 = dir.normalize(sin_theta*widths.second*0.5).rotateRight();

            if(endpts.first.y < endpts.second.y) {
                // pt0 is top
                candidates.push_back(Quad(endpts.second - w2, endpts.first - w1, endpts.first + w1, endpts.second + w2));
            }
            else {
                candidates.push_back(Quad(endpts.first - w1, endpts.second - w2, endpts.second + w2, endpts.first + w1));
            }
        }
    }

    //cout << "candidates: " << candidates.size() << endl;

    removeInvalid(candidates);

    // OVERLAP CHECK
    mergeClose(candidates, 1.0);

    posts = assignGoals(candidates);

    //Improves bottom centre estimate using vertical transitions
    BOOST_FOREACH(ColourSegment& v, v_segments) {
        BOOST_FOREACH(Goal g, posts) {
            if(v.getEnd().y > g.getLocationPixels().y)
                g.setBase(v.getEnd());
        }
    }

#if VISION_GOAL_VERBOSITY > 0
    DataWrapper::getInstance()->debugPublish(DBID_GOALS_RANSAC_CENTRES, posts);
#endif

    return posts;
}
