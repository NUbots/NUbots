#include "goaldetectorhistogram.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/VisionFieldObjects/beacon.h"

#include <limits>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/foreach.hpp>



using namespace boost::accumulators;

GoalDetectorHistogram::GoalDetectorHistogram()
{
}

void GoalDetectorHistogram::run()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    const vector<ColourSegment>& h_segments = VisionBlackboard::getInstance()->getHorizontalTransitions(VisionFieldObject::GOAL_COLOUR);
    const vector<ColourSegment>& v_segments = VisionBlackboard::getInstance()->getVerticalTransitions(VisionFieldObject::GOAL_COLOUR);

    vector<Quad> posts = detectQuads(h_segments, v_segments);

    cout << "candidates: " << posts.size() << endl;

    removeInvalidPosts(posts);
    cout << "after invalid: " << posts.size() << endl;

    // OVERLAP CHECK
    overlapCheck(posts);
    cout << "after overlap: " << posts.size() << endl;

    // DENSITY CHECK - fix later: use segment lengths and scanline spacing to estimate rather than
    //                 re-accessing the image to calculate fully
    //DensityCheck(&yellow_posts, &img, &lut, VisionConstants::GOAL_MIN_PERCENT_YELLOW);

    // CREATE POST OBJECTS
    if (posts.size() != 2) {
        //unable to identify which post is which
        //setting all to unknown
        BOOST_FOREACH(Quad q, posts) {
            vbb->addGoal(Goal(VisionFieldObject::GOAL_U, q));
        }
    }
    else {
        //there are exactly two posts, identify each as left or right
        Quad post1 = posts.at(0),
             post2 = posts.at(1);

        //calculate separation between posts
        int pos1 = std::min(post1.getTopRight().x, post2.getTopRight().x);      // inside right
        int pos2 = std::max(post1.getBottomLeft().x, post2.getBottomLeft().x);  // inside left

        //only publish if the posts are far enough apart
        if(std::abs(pos2 - pos1) >= VisionConstants::MIN_GOAL_SEPARATION) {
            //flip if necessary
            if (post1.getCentre().x > post2.getCentre().x) {
                Quad temp = post2;
                post2 = post1;
                post1 = temp;
            }

            //create left and right goals
            Goal post_left(VisionFieldObject::GOAL_L, post1);
            Goal post_right(VisionFieldObject::GOAL_R, post2);
            //add them to the vision blackboard
            vbb->addGoal(post_left);
            vbb->addGoal(post_right);
        }
    }
}

void GoalDetectorHistogram::DensityCheck(vector<Quad>* posts, NUImage* img, const LookUpTable* lut, const float PERCENT_REQUIRED)
{
    //cout << "PERCENT_REQUIRED: " << PERCENT_REQUIRED << endl;

    vector<Quad>::iterator it = posts->begin();
    while (it < posts->end()) {
        Quad candidate = *it;
        int left = candidate.getBottomLeft().x,
            right = candidate.getTopRight().x,
            top = candidate.getTopRight().y,
            bottom = candidate.getBottomLeft().y;

        //cout << "LEFT: " << left << "\tRIGHT: " << right << "\tTOP: " << top << "\tBOTTOM: " << bottom << endl;

        int count = 0;

        for (int x = left; x < right; x++) {
            for (int y = top; y < bottom; y++) {
                if (ClassIndex::getColourFromIndex(lut->classifyPixel((*img)(x, y))) == ClassIndex::yellow)
                    count++;
            }
        }
        if (((double)count)/((right-left)*(bottom-top)) < PERCENT_REQUIRED) {
            it = posts->erase(it);
            #if VISION_FIELDOBJECT_VERBOSITY > 1
                debug << "GoalDetectorHistogram::yellowDensityCheck - goal thrown out on percentage contained yellow" << endl;
            #endif
        }
        else {
            it++;
        }
    }
}


void GoalDetectorHistogram::removeInvalidPosts(vector<Quad>& posts)
{
    vector<Quad>::iterator it = posts.begin();
    while (it != posts.end()) {
        Quad candidate = *it;
        double height = candidate.getAverageHeight();
        double width = candidate.getAverageWidth();
        if (width < VisionConstants::MIN_GOAL_WIDTH)
            it = posts.erase(it);
        else if (height/width < VisionConstants::GOAL_HEIGHT_TO_WIDTH_RATIO_LOW || height/width > VisionConstants::GOAL_HEIGHT_TO_WIDTH_RATIO_HIGH)
            it = posts.erase(it);
        else
            it++;
    }
}

void GoalDetectorHistogram::overlapCheck(vector<Quad>& posts)
{
    //REPLACE THIS WITH SOMETHING THAT MERGES OVERLAPPING POSTS - OR SHOULD WE NOT EVEN GET OVERLAPPING POSTS
    vector<Quad>::iterator it_a = posts.begin(),
                           it_b;
    while(it_a != posts.end()) {
        it_b = it_a + 1;
        while(it_b != posts.end()) {
            int a_l = it_a->getLeft(),
                a_r = it_a->getRight(),
                b_l = it_b->getLeft(),
                b_r = it_b->getRight();
            //if either of the second posts edges are within the first post remove it
            if( (a_l >= b_l && a_l <= b_r) || (a_r >= b_l && a_r <= b_r) )
                it_b = posts.erase(it_b);
            else
                it_b++;
        }
        it_a++;
    }
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

Vector2<float> GoalDetectorHistogram::calculateSegmentLengthStatistics(const vector<ColourSegment> segments)
{
    accumulator_set<float, stats<tag::mean, tag::variance> > acc;

    BOOST_FOREACH(ColourSegment seg, segments) {
        acc(seg.getLength());
    }

    return Vector2<float>(mean(acc), sqrt(variance(acc)));
}
