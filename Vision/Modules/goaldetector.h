/*! @brief An interface definition for goal detection algorithms.
*/

#ifndef GOALDETECTOR_H
#define GOALDETECTOR_H

#include "Vision/VisionTypes/quad.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTools/lookuptable.h"
#include "Vision/VisionTypes/coloursegment.h"
#include <list>
#include <vector>

using std::vector;
using std::list;

class GoalDetector
{
public:
    GoalDetector();
    virtual ~GoalDetector();
    virtual vector<Goal> run() = 0;

protected:
    //checks
    void removeInvalid(list<Quad> &posts);
    void mergeClose(list<Quad> &posts, double width_multiple_to_merge);

    //generic
    Vector2<double> calculateSegmentLengthStatistics(const vector<ColourSegment> segments);

    vector<Goal> assignGoals(const list<Quad>& candidates) const;
};

#endif // GOALDETECTOR_H
