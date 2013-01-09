/*! @brief Detects goals from yellow transitions using a 1D histogram.
*/

#ifndef TRANSITIONHISTOGRAMMING1D_H
#define TRANSITIONHISTOGRAMMING1D_H

#include <stdio.h>
#include <iostream>

#include "Vision/Modules/goaldetector.h"
#include "Vision/VisionTypes/quad.h"
#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/VisionTools/lookuptable.h"
#include "Vision/VisionTypes/histogram1D.h"

using namespace std;

class TransitionHistogramming1D : public GoalDetector
{
public:
    TransitionHistogramming1D();
    virtual void detectGoals();
private:
    vector<Quad> detectGoal();
    Histogram1D mergePeaks(Histogram1D hist, int minimum_threshold);
    vector<Quad> generateCandidates(const Histogram1D& start, const Histogram1D& end,
                                    const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments,
                                    int peak_threshold, float similarity_threshold);
    Quad generateCandidate(Bin start, Bin end, const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments);

    //checks
    void DensityCheck(bool yellow, bool beacon, vector<Quad>* posts, NUImage* img, const LookUpTable* lut, const float PERCENT_REQUIRED);
    void removeInvalidPosts(vector<Quad>* posts);
    void overlapCheck(vector<Quad>* posts);

    //minor methods
    bool checkBinSimilarity(Bin b1, Bin b2, float allowed_dissimilarity);
};

#endif // TRANSITIONHISTOGRAMMING1D_H
