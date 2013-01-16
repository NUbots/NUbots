#ifndef GOALDETECTORHISTOGRAM2_H
#define GOALDETECTORHISTOGRAM2_H

#include <stdio.h>
#include <iostream>

#include "Vision/Modules/goaldetector.h"
#include "Vision/VisionTypes/quad.h"
#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/VisionTools/lookuptable.h"
#include "Vision/VisionTypes/histogram1D.h"

using namespace std;

class GoalDetectorHistogram2 : public GoalDetector
{
public:
    GoalDetectorHistogram2();
    virtual void run();
private:
    vector<Quad> detectQuads(const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments);
    Histogram1D mergePeaks(Histogram1D hist, int minimum_threshold);
    vector<Quad> generateCandidates(const Histogram1D& hist,
                                    const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments,
                                    int peak_threshold);
    Quad makeQuad(Bin bin, const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments);

    Vector2<float> calculateSegmentLengthStatistics(const vector<ColourSegment> segments);

    //checks
    void DensityCheck(vector<Quad>* posts, NUImage* img, const LookUpTable* lut, const float PERCENT_REQUIRED);
    void removeInvalidPosts(vector<Quad> &posts);
    void overlapCheck(vector<Quad> &posts);

    //minor methods
    bool checkBinSimilarity(Bin b1, Bin b2, float allowed_dissimilarity);
};

#endif // GOALDETECTORHISTOGRAM2_H
