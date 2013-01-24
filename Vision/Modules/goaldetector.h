/*! @brief An interface definition for goal detection algorithms.
*/

#ifndef GOALDETECTOR_H
#define GOALDETECTOR_H

#include "Vision/VisionTypes/quad.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTools/lookuptable.h"
#include "Vision/VisionTypes/coloursegment.h"

using namespace std;

class GoalDetector
{
public:
    virtual void run() = 0;

protected:
    //creation of goals from quads
    vector<Goal> assignGoals(const vector<Quad>& candidates) const;

    //checks
    void removeInvalidPosts(vector<Quad> &posts);
    void DensityCheck(vector<Quad>* posts, NUImage* img, const LookUpTable& lut, const float PERCENT_REQUIRED);
    void overlapCheck(vector<Quad> &posts);

    //generic
    Vector2<float> calculateSegmentLengthStatistics(const vector<ColourSegment> segments);
};

#endif // GOALDETECTOR_H
