#ifndef GOALDETECTORHISTOGRAM_H
#define GOALDETECTORHISTOGRAM_H

#include <stdio.h>
#include <iostream>

#include "Vision/Modules/goaldetector.h"
#include "Vision/VisionTypes/histogram1D.h"

using namespace std;

class GoalDetectorHistogram : public GoalDetector
{
public:
    GoalDetectorHistogram();
    virtual void run();
private:
    vector<Quad> detectQuads(const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments);
    Histogram1D mergePeaks(Histogram1D hist, int minimum_threshold);
    vector<Quad> generateCandidates(const Histogram1D& hist,
                                    const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments,
                                    int peak_threshold);
    Quad makeQuad(Bin bin, const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments);


    //minor methods
    bool checkBinSimilarity(Bin b1, Bin b2, float allowed_dissimilarity);
};

#endif // GOALDETECTORHISTOGRAM_H
