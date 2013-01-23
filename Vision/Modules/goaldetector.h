/*! @brief An interface definition for goal detection algorithms.
*/

#ifndef GOALDETECTOR_H
#define GOALDETECTOR_H

#include "Vision/VisionTypes/quad.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"

using namespace std;

class GoalDetector
{
public:
    GoalDetector();

    virtual void run() = 0;

protected:
    vector<Goal> assignGoals(const vector<Quad>& candidates) const;
};

#endif // GOALDETECTOR_H
