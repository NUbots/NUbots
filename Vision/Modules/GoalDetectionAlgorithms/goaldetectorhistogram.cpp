#include "goaldetectorhistogram.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"
#include "debug.h"
#include "debugverbosityvision.h"

#include <limits>

#include <boost/foreach.hpp>


GoalDetectorHistogram::GoalDetectorHistogram() {}
GoalDetectorHistogram::~GoalDetectorHistogram() {}

vector<Goal> GoalDetectorHistogram::run()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    const vector<ColourSegment>& h_segments = vbb->getHorizontalTransitions(GOAL_COLOUR);
    const vector<ColourSegment>& v_segments = vbb->getVerticalTransitions(GOAL_COLOUR);

    list<Quad> quads = detectQuads(h_segments, v_segments);
    vector<Goal> posts;

    removeInvalid(quads);

    // OVERLAP CHECK
    mergeClose(quads, 1.5);

    // DENSITY CHECK - fix later: use segment lengths and scanline spacing to estimate rather than
    //                 re-accessing the image to calculate fully
    //DensityCheck(&yellow_posts, &img, &lut, VisionConstants::GOAL_MIN_PERCENT_YELLOW);

    posts = assignGoals(quads);

#if VISION_GOAL_VERBOSITY > 0
    DataWrapper::getInstance()->debugPublish(DBID_GOALS_HIST, posts);
#endif

    return posts;
}

list<Quad> GoalDetectorHistogram::detectQuads(const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments)
{
    const size_t BINS = 20;
    const double STDDEV_THRESHOLD = 1.5;
    const double MERGE_THRESHOLD = 50.0 / VisionConstants::HORIZONTAL_SCANLINE_SPACING;
    const double CANDIDATE_THRESHOLD = 200.0 / VisionConstants::HORIZONTAL_SCANLINE_SPACING;

    Histogram1D hist(BINS, VisionBlackboard::getInstance()->getImageWidth()/(double)BINS);

    Vector2<double> h_length_stats = calculateSegmentLengthStatistics(h_segments);

    Histogram1D hist2(BINS, VisionBlackboard::getInstance()->getImageWidth()/(double)BINS);
    // fill histogram bins
    BOOST_FOREACH(ColourSegment seg, h_segments) {
        //use stddev throwout to remove topbar segments
        if(seg.getLength() <= h_length_stats.x + STDDEV_THRESHOLD*h_length_stats.y) {
            hist.addToBin(seg.getCentre().x, seg.getLength());
            hist2.addToBins(seg.getStart().x, seg.getEnd().x + 1, 1);
        }
    }

    // use vertical segments as well
    BOOST_FOREACH(ColourSegment seg, v_segments) {
        hist.addToBin(seg.getCentre().x, seg.getLength());
        hist2.addToBin(seg.getCentre().x, seg.getLength());
    }

    Histogram1D h_merged = mergePeaks(hist, MERGE_THRESHOLD);
    Histogram1D h_merged2 = mergePeaks(hist2, MERGE_THRESHOLD);

    DataWrapper::getInstance()->plotHistogram("Before Merge", hist, blue);
    DataWrapper::getInstance()->plotHistogram("Before Merge2", hist2, blue);
    //DataWrapper::getInstance()->plotHistogram("After Merge", h_merged, yellow);
    //DataWrapper::getInstance()->plotHistogram("After Merge2", h_merged2, yellow);

    return generateCandidates(h_merged, h_segments, v_segments, CANDIDATE_THRESHOLD);
}

Histogram1D GoalDetectorHistogram::mergePeaks(Histogram1D hist, int minimum_size)
{
    hist.mergeAdjacentPeaks(minimum_size);
    return hist;
}

list<Quad> GoalDetectorHistogram::generateCandidates(const Histogram1D& hist, const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments, int peak_threshold)
{
    list<Quad> candidates;
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


// BETTER EDGE FITTING METHOD
Quad GoalDetectorHistogram::makeQuad(Bin bin, const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments)
{
    // find bounding box from histogram
    int    left = bin.start,
           right = left + bin.width;

    //just use pure bounding box for now, later use stddev thresholding

    LSFittedLine l, r;
    Point l1, l2, r1, r2;

    //find left and right by fitting lines
    BOOST_FOREACH(const ColourSegment& h, h_segments) {
        //check segment centre is within bin
        if(h.getCentre().x >= left && h.getCentre().x <= right) {
            l.addPoint(h.getStart());
            r.addPoint(h.getEnd());
        }
    }

    if(l.getEndPoints(l1, l2) && r.getEndPoints(r1, r2)) {
        // ensure l1 and r1 are uppermost
        if(l1.y > l2.y) {
            std::swap(l1, l2);
        }
        if(r1.y > r2.y) {
            // l1 and r2 top
            std::swap(r1, r2);
        }

        //find bottom
        BOOST_FOREACH(const ColourSegment& v, v_segments) {
            //check segment centre is within bin
            const Point& p = v.getCentre();
            if(p.x >= left && p.x <= right) {
                // extend lines to new y
                if(r2.y < p.y)
                    r2 = Point(r.findXFromY(p.y), p.y);
                if(l2.y < p.y)
                    l2 = Point(l.findXFromY(p.y), p.y);
            }
        }
        return Quad(l2, l1, r1, r2);
    }
    else {
        //not enough points
        return Quad();
    }
}


// OLD CRAP BOUNDING BOX METHOD

//Quad GoalDetectorHistogram::makeQuad(Bin bin, const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments)
//{
//    // find bounding box from histogram
//    int    left = bin.start,
//           right = left + bin.width,
//           h_min = std::numeric_limits<int>::max(),
//           h_max = 0,
//           v_min = std::numeric_limits<int>::max(),
//           v_max = 0;

//    //just use pure bounding box for now, later use stddev thresholding

//    //find left and right
//    BOOST_FOREACH(ColourSegment seg, h_segments) {
//        //check segment centre is within bin
//        if(seg.getCentre().x >= left && seg.getCentre().x <= right) {
//            int s_x = seg.getStart().x,
//                e_x = seg.getEnd().x,
//                y = seg.getCentre().y;
//            //check segment's left edge
//            if(s_x < h_min)
//                h_min = s_x; //segment h-pos is leftmost, keep track
//            //check segment's right edge
//            if(e_x > h_max)
//                h_max = e_x; //segment h-pos is rightmost, keep track
//            //check vertical
//            if(y < v_min)
//                v_min = y; //segment v-pos is uppermost, keep track
//            if(y > v_max)
//                v_max = y; //segment v-pos is lowermost, keep track
//        }
//    }

//    //find top and bottom
//    BOOST_FOREACH(ColourSegment seg, v_segments) {
//        //check segment centre is within bin
//        if(seg.getCentre().x >= left && seg.getCentre().x <= right) {
//            //check start
//            int s_y = seg.getStart().y,
//                e_y = seg.getEnd().y;

//            if(s_y < v_min)
//                v_min = s_y; //segment is topmost, keep track
//            if(e_y > v_max)
//                v_max = e_y; //segment is bottommost, keep track
//        }
//    }

//    return Quad(Point(h_min, v_max), Point(h_min, v_min), Point(h_max, v_min), Point(h_max, v_max));
//}

