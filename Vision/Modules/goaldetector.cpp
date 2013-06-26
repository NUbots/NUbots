#include "goaldetector.h"
#include "Vision/visionconstants.h"
#include <boost/foreach.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include "Vision/visionblackboard.h"
#include "Vision/VisionTools/transformer.h"

using namespace boost::accumulators;

GoalDetector::GoalDetector() {}
GoalDetector::~GoalDetector() {}

//void GoalDetector::relabel(std::vector<Goal>& goals, std::vector<Obstacle> obstacles, COLOUR_CLASS team_colour, COLOUR_CLASS opponent_colour) const
//{
//    if(goals.size() == 2) {
//        int left_index, right_index;
//        if(goals[0].getID() == GOAL_L && goals[1].getID() == GOAL_R) {
//            left_index = 0;
//            right_index = 1;
//        }
//        else if(goals[0].getID() == GOAL_R && goals[1].getID() == GOAL_L) {
//            left_index = 1;
//            right_index = 0;
//        }
//        else {
//            return;
//        }

//        int team_count = 0;
//        int opponent_count = 0;
//        double avg_goal_dist = 0.5 * (goals[0].getLocation().neckRelativeRadial.x + goals[1].getLocation().neckRelativeRadial.x);
//        // Count number of team or opponent obstacles in goals
//        for(const Obstacle& obst : obstacles) {
//            if(obst.getLocationPixels().x > goals[left_index].getLocationPixels().x &&
//               obst.getLocationPixels().x < goals[right_index].getLocationPixels().x &&
//               obst.getLocation().neckRelativeRadial.x < avg_goal_dist + 100 &&
//               obst.getLocation().neckRelativeRadial.x > avg_goal_dist - 100 )
//            {
//                //obstacle is within reasonable distance of goal centre
//                if(obst.m_colour == team_colour)
//                    team_count++;
//                else if(obst.m_colour == opponent_colour)
//                    opponent_count++;
//            }
//        }

//        if(team_count == 1 && opponent_count == 0) {
//            //own goal
//        }
//        else if(team_count == 0 && opponent_count == 1) {
//            //opponent goal
//        }
//    }
//}

void GoalDetector::relabel(std::vector<Goal>& goals) const
{
    // we can only do this stuff if we have two goals
    if(goals.size() == 2) {
        //std::cout << "GOAL HACK STARTING" << std::endl;
        // find left from right
        int left_index, right_index;
        if(goals[0].getID() == GOAL_L && goals[1].getID() == GOAL_R) {
            left_index = 0;
            right_index = 1;
        }
        else if(goals[0].getID() == GOAL_R && goals[1].getID() == GOAL_L) {
            left_index = 1;
            right_index = 0;
        }
        else {
            //std::cout << "GOAL HACK LEFT AND RIGHT NOT SET - EXITTING" << std::endl;
            return;
        }

        double avg_dist = 0.5 * (goals[0].width_dist + goals[1].width_dist);

        //std::cout << "GOAL HACK avg_dist: " << avg_dist << std::endl;
        // only apply for distance goals
        if(avg_dist > 250) {
            const Transformer& tran = VisionBlackboard::getInstance()->getTransformer();
            double pix_dist = (goals[0].m_location.screenCartesian - goals[1].m_location.screenCartesian).abs();
            double between_dist = VisionConstants::DISTANCE_BETWEEN_POSTS*tran.getCameraDistanceInPixels()/pix_dist;
            double d0 = between_dist + goals[0].width_dist - avg_dist;
            double d1 = between_dist + goals[1].width_dist - avg_dist;

            //std::cout << "GOAL HACK relabelling original. d0: " << goals[0].width_dist << " d1: " << goals[1].width_dist << std::endl;
            //std::cout << "GOAL HACK relabelling new. d0: " << d0 << " d1: " << d1 << std::endl;
            // check the below is actually changing the vals
            tran.calculateRepresentationsFromPixelLocation(goals[0].m_location, true, d0);
            tran.calculateRepresentationsFromPixelLocation(goals[1].m_location, true, d1);
        }

        // now label blue and yellow based on white background
        // only if the config parameter is set to
        if(VisionConstants::WHITE_SIDE_IS_BLUE >= 0) {
            //std::cout << "GOAL HACK WHITE PART STARTING" << std::endl;
//            Goal& left_goal = goals[left_index];
//            Goal& right_goal = goals[right_index];
//            double left = left_goal.m_location.screenCartesian.x;
//            double right = right_goal.m_location.screenCartesian.x
//            double top = std::min(left_goal.m_location.screenCartesian.y - left_goal.m_size_on_screen.y,
//                                  right_goal.m_location.screenCartesian.y - right_goal.m_size_on_screen.y);
//            double bottom = std::min(left_goal.m_location.screenCartesian.y - left_goal.m_size_on_screen.y,
//                                  right_goal.m_location.screenCartesian.y - right_goal.m_size_on_screen.y);
//            Point bottomright = right.m_location.screenCartesian;
//            double left = topleft.x + bottom
            const GreenHorizon& gh = VisionBlackboard::getInstance()->getGreenHorizon();
            const std::vector<ColourSegment>& segs = VisionBlackboard::getInstance()->getAllTransitions(LINE_COLOUR);

            double white_count = 0;

            for(const ColourSegment& s : segs) {
                if( ! gh.isBelowHorizon(Point(s.getCentre().x, std::min(s.getEnd().y, s.getStart().y)))) {
                    white_count += s.getLength();
                }
            }
            //std::cout << "GOAL HACK WHITE count: " << white_count << std::endl;

            double THRESHOLD = 500;
            if(white_count > THRESHOLD) {
                if(VisionConstants::WHITE_SIDE_IS_BLUE) {
                    //std::cout << "GOAL HACK WHITE labelling blue: " << white_count << std::endl;
                    goals[left_index].m_id = GOAL_B_L;
                    goals[right_index].m_id = GOAL_B_R;
                }
                else {
                    //std::cout << "GOAL HACK WHITE labelling yellow: " << white_count << std::endl;
                    goals[left_index].m_id = GOAL_Y_L;
                    goals[right_index].m_id = GOAL_Y_R;
                }
            }
        }
    }
}

void GoalDetector::removeInvalid(std::list<Quad>& posts)
{
    std::list<Quad>::iterator it = posts.begin();
    while (it != posts.end()) {
        //remove all posts whos' aspect ratios are too low
        if ( it->aspectRatio() < VisionConstants::GOAL_HEIGHT_TO_WIDTH_RATIO_MIN)
            it = posts.erase(it);
        else
            it++;
    }
}

void GoalDetector::mergeClose(std::list<Quad> &posts, double width_multiple_to_merge)
{
    std::list<Quad>::iterator a = posts.begin(),
                         b;
    while(a != posts.end()) {
        b = a;
        b++;
        while(b != posts.end()) {
            // if the posts overlap
            // or if their centres are horizontally closer than the largest widths multiplied by width_multiple_to_merge
            if(a->overlapsHorizontally(*b) ||
               std::abs( a->getCentre().x - b->getCentre().x ) <= std::max(a->getAverageWidth(), b->getAverageWidth())*width_multiple_to_merge) {
                // get outer lines
                Point tl( std::min(a->getTopLeft().x, b->getTopLeft().x)         , std::min(a->getTopLeft().y, b->getTopLeft().y) ),
                      tr( std::max(a->getTopRight().x, b->getTopRight().x)       , std::min(a->getTopRight().y, b->getTopRight().y) ),
                      bl( std::min(a->getBottomLeft().x, b->getBottomLeft().x)   , std::max(a->getBottomLeft().y, b->getBottomLeft().y) ),
                      br( std::max(a->getBottomRight().x, b->getBottomRight().x) , std::max(a->getBottomRight().y, b->getBottomRight().y) );

                //replace original two quads with the new one
                a->set(bl, tl, tr, br);
                b = posts.erase(b);
            }
            else {
                b++;
            }
        }
        a++;
    }
}

Vector2<double> GoalDetector::calculateSegmentLengthStatistics(const std::vector<ColourSegment> segments)
{
    accumulator_set<double, stats<tag::mean, tag::variance> > acc;

    BOOST_FOREACH(ColourSegment seg, segments) {
        acc(seg.getLength());
    }

    return Vector2<double>(mean(acc), sqrt(variance(acc)));
}

std::vector<Goal> GoalDetector::assignGoals(const std::list<Quad>& candidates) const
{
    std::vector<Goal> goals;
    if(candidates.size() == 2) {
        //there are exactly two candidates, identify each as left or right
        Quad post1 = candidates.front(),
             post2 = candidates.back();

        //calculate separation between candidates
        double pos1 = std::min(post1.getRight(), post2.getRight()),      // inside right
               pos2 = std::max(post1.getLeft(), post2.getLeft());  // inside left

        //only publish if the candidates are far enough apart
        if(std::abs(pos2 - pos1) >= VisionConstants::MIN_GOAL_SEPARATION) {
            //flip if necessary
            if (post1.getCentre().x > post2.getCentre().x) {
                goals.push_back(Goal(GOAL_L, post2));
                goals.push_back(Goal(GOAL_R, post1));
            }
            else {
                goals.push_back(Goal(GOAL_L, post1));
                goals.push_back(Goal(GOAL_R, post2));
            }
        }
        else {
            //should merge
        }
    }
    else {
        //unable to identify which post is which
        //setting all to unknown
        for(Quad q : candidates) {
            goals.push_back(Goal(GOAL_U, q));
        }
    }
    return goals;
}
