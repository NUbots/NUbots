#include "goaldetectorhistogram.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"

#include <limits>

#include <boost/foreach.hpp>


GoalDetectorHistogram::GoalDetectorHistogram()
{
}

vector<Goal> GoalDetectorHistogram::run()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    const vector<ColourSegment>& h_segments = vbb->getHorizontalTransitions(GOAL_COLOUR);
    const vector<ColourSegment>& v_segments = vbb->getVerticalTransitions(GOAL_COLOUR);

    vector<Quad> posts = detectQuads(h_segments, v_segments);

    removeInvalidPosts(posts);

    // OVERLAP CHECK
    overlapCheck(posts);

    // DENSITY CHECK - fix later: use segment lengths and scanline spacing to estimate rather than
    //                 re-accessing the image to calculate fully
    //DensityCheck(&yellow_posts, &img, &lut, VisionConstants::GOAL_MIN_PERCENT_YELLOW);

    return assignGoals(posts);
}

vector<Quad> GoalDetectorHistogram::detectQuads(const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments)
{
    const int BINS = 20;
    const int MERGE_THRESHOLD = 50;
    const float STDDEV_THRESHOLD = 1.5;
    const int CANDIDATE_THRESHOLD = 200;

    Histogram1D hist(BINS, VisionBlackboard::getInstance()->getImageWidth()/(double)BINS);

    Vector2<float> h_length_stats = calculateSegmentLengthStatistics(h_segments);

    // fill histogram bins
    BOOST_FOREACH(ColourSegment seg, h_segments) {
        //use stddev throwout to remove topbar segments
        if(seg.getLength() <= h_length_stats.x + STDDEV_THRESHOLD*h_length_stats.y)
            hist.addToBin(seg.getCentre().x, seg.getLength());
    }

    // use vertical segments as well
    BOOST_FOREACH(ColourSegment seg, v_segments) {
        hist.addToBin(seg.getCentre().x, seg.getLength());
    }

    Histogram1D h_merged = mergePeaks(hist, MERGE_THRESHOLD);

    return generateCandidates(h_merged, h_segments, v_segments, CANDIDATE_THRESHOLD);
}

Histogram1D GoalDetectorHistogram::mergePeaks(Histogram1D hist, int minimum_size)
{
    hist.mergeAdjacentPeaks(minimum_size);
    return hist;
}

vector<Quad> GoalDetectorHistogram::generateCandidates(const Histogram1D& hist,
                                                           const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments,
                                                           int peak_threshold)
{
    vector<Quad> candidates;
    vector<Bin>::const_iterator b_it;

    for (b_it = hist.begin(); b_it != hist.end(); b_it++) {
        if(b_it->value >= peak_threshold) {
            //consider this peak - generate candidate
            candidates.push_back(makeQuad(*b_it, h_segments, v_segments));
        }
    }

    return candidates;
}


bool GoalDetectorHistogram::checkBinSimilarity(Bin b1, Bin b2, float allowed_dissimilarity)
{
    return b1.value*(1+allowed_dissimilarity) >= b2.value && b1.value*(1-allowed_dissimilarity) <= b2.value;
}

Quad GoalDetectorHistogram::makeQuad(Bin bin, const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments)
{
    // find bounding box from histogram
    int    left = bin.start,
           right = left + bin.width,
           h_min = std::numeric_limits<int>::max(),
           h_max = 0,
           v_min = std::numeric_limits<int>::max(),
           v_max = 0;

    //just use pure bounding box for now, later use stddev thresholding

    //find left and right
    BOOST_FOREACH(ColourSegment seg, h_segments) {
        //check segment centre is within bin
        if(seg.getCentre().x >= left && seg.getCentre().x <= right) {
            int s_x = seg.getStart().x,
                e_x = seg.getEnd().x,
                y = seg.getCentre().y;
            //check segment's left edge
            if(s_x < h_min)
                h_min = s_x; //segment h-pos is leftmost, keep track
            //check segment's right edge
            if(e_x > h_max)
                h_max = e_x; //segment h-pos is rightmost, keep track
            //check vertical
            if(y < v_min)
                v_min = y; //segment v-pos is uppermost, keep track
            if(y > v_max)
                v_max = y; //segment v-pos is lowermost, keep track
        }
    }

    //find top and bottom
    BOOST_FOREACH(ColourSegment seg, v_segments) {
        //check segment centre is within bin
        if(seg.getCentre().x >= left && seg.getCentre().x <= right) {
            //check start
            int s_y = seg.getStart().y,
                e_y = seg.getEnd().y;

            if(s_y < v_min)
                v_min = s_y; //segment is bottommost, keep track
            if(e_y > v_max)
                v_max = e_y; //segment is uppermost, keep track
        }
    }

    return Quad(h_min, v_min, h_max, v_max);
}
