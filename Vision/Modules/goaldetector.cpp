#include "goaldetector.h"
#include "Vision/visionconstants.h"
#include <boost/foreach.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

using namespace boost::accumulators;

GoalDetector::GoalDetector() {}
GoalDetector::~GoalDetector() {}

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
