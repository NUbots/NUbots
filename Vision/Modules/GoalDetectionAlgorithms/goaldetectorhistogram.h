/*! @brief Detects goals from yellow transitions using a 1D histogram.
*/

#ifndef GOALDETECTORHISTOGRAM_H
#define GOALDETECTORHISTOGRAM_H

#include <stdio.h>
#include <iostream>

#include "Vision/Modules/goaldetector.h"
#include "Vision/VisionTypes/quad.h"
#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/VisionTools/lookuptable.h"
#include "Vision/VisionTypes/histogram1D.h"

using namespace std;

class GoalDetectorHistogram : public GoalDetector
{
public:
    GoalDetectorHistogram();
    virtual void run();
private:
    vector<Quad> detectQuads();
    Histogram1D mergePeaks(Histogram1D hist, int minimum_threshold);
    vector<Quad> generateCandidates(const Histogram1D& start, const Histogram1D& end,
                                    const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments,
                                    int peak_threshold, float similarity_threshold);
    Quad makeQuad(Bin start, Bin end, const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments);

    //checks
    void DensityCheck(vector<Quad>* posts, NUImage* img, const LookUpTable* lut, const float PERCENT_REQUIRED);
    void removeInvalidPosts(vector<Quad> &posts);
    void overlapCheck(vector<Quad> &posts);

    //minor methods
    bool checkBinSimilarity(Bin b1, Bin b2, float allowed_dissimilarity);
};

#endif // GOALDETECTORHISTOGRAM_H
