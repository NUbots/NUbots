/*! @brief An interface definition for goal detection algorithms.
*/

#ifndef GOALDETECTOR_H
#define GOALDETECTOR_H

#include "Vision/VisionTypes/quad.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/VisionFieldObjects/obstacle.h"
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
    virtual std::vector<Goal> run() = 0;

    virtual void relabel(std::vector<Goal>& goals) const;

protected:
    //checks
    void removeInvalid(std::list<Quad> &posts);
    void mergeClose(std::list<Quad> &posts, double width_multiple_to_merge);

    //generic
    Vector2<double> calculateSegmentLengthStatistics(const std::vector<ColourSegment> segments);

    std::vector<Goal> assignGoals(const std::list<Quad>& candidates) const;
};

#endif // GOALDETECTOR_H
