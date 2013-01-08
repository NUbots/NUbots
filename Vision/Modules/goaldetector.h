/*! @brief An interface definition for goal detection algorithms.
*/

#ifndef GOALDETECTOR_H
#define GOALDETECTOR_H

using namespace std;

class GoalDetector
{
public:
    GoalDetector();

    virtual void detectGoals() = 0;
};

#endif // GOALDETECTOR_H
