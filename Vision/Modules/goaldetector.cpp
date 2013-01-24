#include "goaldetector.h"
#include "Vision/visionconstants.h"
#include <boost/foreach.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

using namespace boost::accumulators;

vector<Goal> GoalDetector::assignGoals(const vector<Quad>& candidates) const
{
    vector<Goal> goals;
    if(candidates.size() == 2) {
        //there are exactly two candidates, identify each as left or right
        Quad post1 = candidates.at(0),
             post2 = candidates.at(1);

        //calculate separation between candidates
        double pos1 = std::min(post1.getTopRight().x, post2.getTopRight().x),      // inside right
               pos2 = std::max(post1.getBottomLeft().x, post2.getBottomLeft().x);  // inside left

        //only publish if the candidates are far enough apart
        if(std::abs(pos2 - pos1) >= VisionConstants::MIN_GOAL_SEPARATION) {
            //flip if necessary
            if (post1.getCentre().x > post2.getCentre().x) {
                goals.push_back(Goal(VisionFieldObject::GOAL_L, post2));
                goals.push_back(Goal(VisionFieldObject::GOAL_R, post1));
            }
            else {
                goals.push_back(Goal(VisionFieldObject::GOAL_L, post1));
                goals.push_back(Goal(VisionFieldObject::GOAL_R, post2));
            }
        }
    }
    else {
        //unable to identify which post is which
        //setting all to unknown
        BOOST_FOREACH(Quad q, candidates) {
            goals.push_back(Goal(VisionFieldObject::GOAL_U, q));
        }
    }
    return goals;
}

void GoalDetector::removeInvalidPosts(vector<Quad>& posts)
{
    vector<Quad>::iterator it = posts.begin();
    while (it != posts.end()) {
        Quad candidate = *it;
        double height = candidate.getAverageHeight();
        double width = candidate.getAverageWidth();
        if (width < VisionConstants::MIN_GOAL_WIDTH)
            it = posts.erase(it);
        else if (height/width < VisionConstants::GOAL_HEIGHT_TO_WIDTH_RATIO_LOW || height/width > VisionConstants::GOAL_HEIGHT_TO_WIDTH_RATIO_HIGH)
            it = posts.erase(it);
        else
            it++;
    }
}

void GoalDetector::DensityCheck(vector<Quad>* posts, NUImage* img, const LookUpTable& lut, const float PERCENT_REQUIRED)
{
    //cout << "PERCENT_REQUIRED: " << PERCENT_REQUIRED << endl;

    vector<Quad>::iterator it = posts->begin();
    while (it < posts->end()) {
        Quad candidate = *it;
        int left = candidate.getBottomLeft().x,
            right = candidate.getTopRight().x,
            top = candidate.getTopRight().y,
            bottom = candidate.getBottomLeft().y;

        //cout << "LEFT: " << left << "\tRIGHT: " << right << "\tTOP: " << top << "\tBOTTOM: " << bottom << endl;

        int count = 0;

        for (int x = left; x < right; x++) {
            for (int y = top; y < bottom; y++) {
                if (ClassIndex::getColourFromIndex(lut.classifyPixel((*img)(x, y))) == ClassIndex::yellow)
                    count++;
            }
        }
        if (((double)count)/((right-left)*(bottom-top)) < PERCENT_REQUIRED) {
            it = posts->erase(it);
            #if VISION_FIELDOBJECT_VERBOSITY > 1
                debug << "GoalDetectorHistogram::yellowDensityCheck - goal thrown out on percentage contained yellow" << endl;
            #endif
        }
        else {
            it++;
        }
    }
}

void GoalDetector::overlapCheck(vector<Quad>& posts)
{
    //REPLACE THIS WITH SOMETHING THAT MERGES OVERLAPPING POSTS - OR SHOULD WE NOT EVEN GET OVERLAPPING POSTS
    vector<Quad>::iterator it_a = posts.begin(),
                           it_b;
    while(it_a != posts.end()) {
        it_b = it_a + 1;
        while(it_b != posts.end()) {
            int a_l = it_a->getLeft(),
                a_r = it_a->getRight(),
                b_l = it_b->getLeft(),
                b_r = it_b->getRight();
            //if either of the second posts edges are within the first post remove it
            if( (a_l >= b_l && a_l <= b_r) || (a_r >= b_l && a_r <= b_r) )
                it_b = posts.erase(it_b);
            else
                it_b++;
        }
        it_a++;
    }
}

Vector2<float> GoalDetector::calculateSegmentLengthStatistics(const vector<ColourSegment> segments)
{
    accumulator_set<float, stats<tag::mean, tag::variance> > acc;

    BOOST_FOREACH(ColourSegment seg, segments) {
        acc(seg.getLength());
    }

    return Vector2<float>(mean(acc), sqrt(variance(acc)));
}
