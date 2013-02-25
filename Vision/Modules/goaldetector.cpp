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

void GoalDetector::removeInvalid(list<Quad>& posts)
{
    list<Quad>::iterator it = posts.begin();
    while (it != posts.end()) {
        //remove all posts whos aspect ratio is too low
        if (it->getAverageHeight() / it->getAverageWidth() < VisionConstants::GOAL_HEIGHT_TO_WIDTH_RATIO_MIN)
            it = posts.erase(it);
        else
            it++;
    }
}

void GoalDetector::mergeOverlapping(list<Quad> &posts) {
    list<Quad>::iterator a = posts.begin(),
                         b;
    while(a != posts.end()) {
        b = a;
        b++;
        while(b != posts.end()) {
            double distance = (a->getCentre() - b->getCentre()).abs();
            if( distance < a->getAverageWidth() || distance < b->getAverageWidth() ) {
                // get outer lines
                Point tl( min(a->getTopLeft().x, b->getTopLeft().x)         , min(a->getTopLeft().y, b->getTopLeft().y) ),
                      tr( max(a->getTopRight().x, b->getTopRight().x)       , min(a->getTopRight().y, b->getTopRight().y) ),
                      bl( min(a->getBottomLeft().x, b->getBottomLeft().x)   , max(a->getBottomLeft().y, b->getBottomLeft().y) ),
                      br( max(a->getBottomRight().x, b->getBottomRight().x) , max(a->getBottomRight().y, b->getBottomRight().y) );

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

Vector2<double> GoalDetector::calculateSegmentLengthStatistics(const vector<ColourSegment> segments)
{
    accumulator_set<double, stats<tag::mean, tag::variance> > acc;

    BOOST_FOREACH(ColourSegment seg, segments) {
        acc(seg.getLength());
    }

    return Vector2<double>(mean(acc), sqrt(variance(acc)));
}

vector<Goal> GoalDetector::assignGoals(const list<Quad>& candidates) const
{
    vector<Goal> goals;
    if(candidates.size() == 2) {
        //there are exactly two candidates, identify each as left or right
        Quad post1 = candidates.front(),
             post2 = candidates.back();

        //calculate separation between candidates
        double pos1 = std::min(post1.getTopRight().x, post2.getTopRight().x),      // inside right
               pos2 = std::max(post1.getBottomLeft().x, post2.getBottomLeft().x);  // inside left

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
    }
    else {
        //unable to identify which post is which
        //setting all to unknown
        BOOST_FOREACH(Quad q, candidates) {
            goals.push_back(Goal(GOAL_U, q));
        }
    }
    return goals;
}
