#include "goaldetector.h"
#include "Vision/visionconstants.h"
#include <boost/foreach.hpp>

GoalDetector::GoalDetector()
{    
}

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
