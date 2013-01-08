/*! @brief Detects goals from yellow transitions using a 1D histogram.
*/

#ifndef TRANSITIONHISTOGRAMMING1D_H
#define TRANSITIONHISTOGRAMMING1D_H

#include <stdio.h>
#include <iostream>

#include "Vision/Modules/goaldetector.h"
#include "Vision/VisionTypes/histogram1D.h"

using namespace std;

class TransitionHistogramming1D : public GoalDetector
{
public:
    TransitionHistogramming1D();
    virtual void detectGoals();
private:
    void detectGoal(vector<Quad>* candidates);
    Histogram1D mergePeaks(Histogram1D hist, int minimum_threshold);
    void DensityCheck(bool yellow, bool beacon, vector<Quad>* posts, NUImage* img, const LookUpTable* lut, const float PERCENT_REQUIRED);
    void removeInvalidPosts(vector<Quad>* posts);
    void overlapCheck(vector<Quad>* posts);
};

#endif // TRANSITIONHISTOGRAMMING1D_H
