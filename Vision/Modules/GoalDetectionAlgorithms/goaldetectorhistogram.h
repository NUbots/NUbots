#ifndef GOALDETECTORHISTOGRAM_H
#define GOALDETECTORHISTOGRAM_H

#include <stdio.h>
#include <iostream>

#include "Vision/Modules/goaldetector.h"
#include "Vision/VisionTypes/histogram1d.h"

using std::vector;
using std::list;

class GoalDetectorHistogram : public GoalDetector
{
public:
    GoalDetectorHistogram();
    ~GoalDetectorHistogram();
    virtual std::vector<Goal> run();
private:
    std::list<Quad> detectQuads(const std::vector<ColourSegment>& h_segments, const std::vector<ColourSegment>& v_segments);
    Histogram1D mergePeaks(Histogram1D hist, int minimum_threshold);
    std::list<Quad> generateCandidates(const Histogram1D& hist,
                                  const std::vector<ColourSegment>& h_segments, const std::vector<ColourSegment>& v_segments,
                                  int peak_threshold);
    Quad makeQuad(Bin bin, const std::vector<ColourSegment>& h_segments, const std::vector<ColourSegment>& v_segments);

    //minor methods
    bool checkBinSimilarity(Bin b1, Bin b2, float allowed_dissimilarity);
};

#endif // GOALDETECTORHISTOGRAM_H
