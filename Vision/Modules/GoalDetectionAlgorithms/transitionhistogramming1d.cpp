#include "transitionhistogramming1d.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/VisionFieldObjects/beacon.h"

#include <limits>

#include <boost/foreach.hpp>

TransitionHistogramming1D::TransitionHistogramming1D()
{
}

void TransitionHistogramming1D::detectGoals()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    NUImage img = vbb->getOriginalImage();
    const LookUpTable& lut = vbb->getLUT();

    vector<Quad> yellow_posts = detectPosts();

    removeInvalidPosts(&yellow_posts);

    // OVERLAP CHECK
    overlapCheck(&yellow_posts);

    // DENSITY CHECK - fix later: use segment lengths and quad area to calculate rather than
    //                  re-accessing the image
    DensityCheck(true, false, &yellow_posts, &img, &lut, VisionConstants::GOAL_MIN_PERCENT_YELLOW);

    // YELLOW POSTS
    if (yellow_posts.size() != 2) {
        BOOST_FOREACH(Quad q, yellow_posts) {
//            Goal post(VisionFieldObject::GOAL_Y_U, q);
//            vbb->addGoal(post);
            vbb->addGoal(Goal(VisionFieldObject::GOAL_Y_U, q));
        }
    }
    else {
        Quad post1 = yellow_posts.at(0),
             post2 = yellow_posts.at(1);

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
            Goal post_left(VisionFieldObject::GOAL_Y_L, post1);
            Goal post_right(VisionFieldObject::GOAL_Y_R, post2);
            //add them to the vision blackboard
            vbb->addGoal(post_left);
            vbb->addGoal(post_right);
        }
    }
}

void TransitionHistogramming1D::DensityCheck(bool yellow, bool beacon, vector<Quad>* posts, NUImage* img, const LookUpTable* lut, const float PERCENT_REQUIRED)
{
    //fix later: use segment lengths and quad area to calculate rather than
    //                  re-accessing the image
    ClassIndex::Colour colour, other_colour;
    if (yellow) {
        colour = ClassIndex::yellow;
        other_colour = ClassIndex::blue;
    }
    else {
        colour = ClassIndex::blue;
        other_colour = ClassIndex::yellow;
    }

    //cout << "PERCENT_REQUIRED: " << PERCENT_REQUIRED << endl;

    vector<Quad>::iterator it = posts->begin();
    while (it < posts->end()) {
        Quad candidate = *it;
        int left, right, top, bottom;
        left = candidate.getBottomLeft().x;
        right = candidate.getTopRight().x;
        top = candidate.getTopRight().y;
        bottom = candidate.getBottomLeft().y;

        //cout << "LEFT: " << left << "\tRIGHT: " << right << "\tTOP: " << top << "\tBOTTOM: " << bottom << endl;

        // should probably fix the cause of this at some point...
        if (left > right) {
            int temp = left;
            left = right;
            right = temp;
        }
        if (top > bottom) {
            int temp = top;
            top = bottom;
            bottom = temp;
        }

        int count = 0;
        int other_count = 0;

        for (int i = left; i < right; i++) {
            for (int j = top; j < bottom; j++) {
                if (ClassIndex::getColourFromIndex(lut->classifyPixel((*img)(i, j))) == colour)
                    count++;
                else if (ClassIndex::getColourFromIndex(lut->classifyPixel((*img)(i, j))) == other_colour)
                    other_count++;
            }
        }
        if (((double)count)/((right-left)*(bottom-top)) < PERCENT_REQUIRED) {
            it = posts->erase(it);
            #if VISION_FIELDOBJECT_VERBOSITY > 1
                debug << "TransitionHistogramming1D::yellowDensityCheck - goal thrown out on percentage contained yellow" << endl;
            #endif
        }
        else if (other_count > 0 && !beacon) {
            it = posts->erase(it);
        }
        else {
            it++;
        }
    }
}


void TransitionHistogramming1D::removeInvalidPosts(vector<Quad>* posts)
{
    vector<Quad>::iterator it = posts->begin();
    while (it < posts->end()) {
        Quad candidate = *it;
        int height = candidate.getHeight();
        int width = candidate.getWidth();
//        int height = candidate.val[3] - candidate.val[1],
//            width = candidate.val[2] - candidate.val[0];
        if (width == 0)
            it = posts->erase(it);
        else if (it->getWidth() < VisionConstants::MIN_GOAL_WIDTH)
            it = posts->erase(it);
        else if (height/width < VisionConstants::GOAL_HEIGHT_TO_WIDTH_RATIO_LOW || height/width > VisionConstants::GOAL_HEIGHT_TO_WIDTH_RATIO_HIGH)
            it = posts->erase(it);
        else
            it++;
    }
}

void TransitionHistogramming1D::overlapCheck(vector<Quad>* posts)
{
    //REPLACE THIS WITH SOMETHING THAT MERGES OVERLAPPING POSTS - OR SHOULD WE NOT EVEN GET OVERLAPPING POSTS
    for (unsigned int i = 0; i < posts->size(); i++) {
        cv::Scalar left = posts->at(i).getAsScalar();
        for (unsigned int j = i+1; j < posts->size(); j++) {
            vector<Quad>::iterator it = posts->begin()+j;
            cv::Scalar right = it->getAsScalar();
            if (left.val[2] >= right.val[0] && left.val[0] <= right.val[2])
                posts->erase(it);
        }
    }
}

vector<Quad> TransitionHistogramming1D::detectPosts()
{
    const vector<ColourSegment>& h_segments = VisionBlackboard::getInstance()->getHorizontalTransitions(VisionFieldObject::GOAL_Y_COLOUR);
    const vector<ColourSegment>& v_segments = VisionBlackboard::getInstance()->getVerticalTransitions(VisionFieldObject::GOAL_Y_COLOUR);

    //vector<PointType> start_trans, end_trans, vert_trans;
    //vector<int> start_lengths, end_lengths;

    const int BINS = 20;
    const int WIDTH = VisionBlackboard::getInstance()->getImageWidth();
    const int BIN_WIDTH = WIDTH/BINS;
    const int MIN_THRESHOLD = 1;
    const float SDEV_THRESHOLD = 0.75;
    const int PEAK_THRESHOLD = 10;
    const float ALLOWED_DISSIMILARITY = 0.5;

    Histogram1D h_start(BINS, BIN_WIDTH),
                h_end(BINS, BIN_WIDTH);

    int MAX_WIDTH = 3;

    // fill histogram bins
    BOOST_FOREACH(ColourSegment seg, h_segments) {
        h_start.addToBin(seg.getStart().x, seg.getLength());
        h_end.addToBin(seg.getEnd().x, seg.getLength());
    }

    Histogram1D h_start_merged = mergePeaks(h_start, MIN_THRESHOLD);
    Histogram1D h_end_merged = mergePeaks(h_end, MIN_THRESHOLD);

    return generateCandidates(h_start_merged, h_end_merged, h_segments, v_segments, PEAK_THRESHOLD, ALLOWED_DISSIMILARITY);
}

Histogram1D TransitionHistogramming1D::mergePeaks(Histogram1D hist, int minimum_size)
{
    hist.mergeAdjacentPeaks(minimum_size);
    return hist;
}

vector<Quad> TransitionHistogramming1D::generateCandidates(const Histogram1D& start, const Histogram1D& end,
                                                           const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments,
                                                           int peak_threshold, float allowed_dissimilarity)
{
    vector<Quad> candidates;

    // pair start transitions and end transitions
    vector<Bin>::const_iterator s_it = start.begin(),
                                e_it = end.begin(),
                                e_holder;

    while (s_it != start.end() && e_it != end.end()) {
        if(s_it->value >= peak_threshold) {
            //consider this peak
            //move the end transition iterator up to the start transition iterator
            //note that the histograms may not line up exactly
            while(e_it->start < s_it->start && e_it != end.end()) {
                e_it++;
            }

            e_holder = e_it; //keep track of where the iterator was

            //now move along the end histogram using e_it until reaching the end or found a bin within allowed_dissimilarity of
            //the current start bin size
            while(e_it != end.end() && !checkBinSimilarity(*e_it, *s_it, allowed_dissimilarity)) {
                e_it++;
            }

            if(e_it != end.end()) {
                //we found a matching bin - generate candidate
                candidates.push_back(generateCandidate(*s_it, *e_it, h_segments, v_segments));

                //move the start iterator until after the end iterator
                while(s_it != start.end() && s_it->start < e_it->start + e_it->width) {
                    s_it++;
                }
            }
            else {
                //no match - move to next start bin
                s_it++;
            }
            e_it = e_holder; //put e_it back to its previous location
        }
        else {
            s_it++;
        }
    }




    return candidates;
}


bool TransitionHistogramming1D::checkBinSimilarity(Bin b1, Bin b2, float allowed_dissimilarity)
{
    return b1.value*(1+allowed_dissimilarity) >= b2.value && b1.value*(1-allowed_dissimilarity) <= b2.value;
}

Quad TransitionHistogramming1D::generateCandidate(Bin start, Bin end, const vector<ColourSegment>& h_segments, const vector<ColourSegment>& v_segments)
{
    // find bounding box from histogram
    int    left = start.start,
           right = end.start + end.width,
           h_min = std::numeric_limits<int>::max(),
           h_max = 0,
           v_min = std::numeric_limits<int>::max(),
           v_max = 0;

    //just use pure bounding box for now, later use stddev thresholding

    //find left and right
    BOOST_FOREACH(ColourSegment seg, h_segments) {
        //check start
        int s_x = seg.getStart().x,
            s_y = seg.getStart().y,
            e_x = seg.getEnd().x,
            e_y = seg.getEnd().y;
        if(s_x >= left && s_x <= right) {
            //segment's left edge is withing the bounding box
            if(s_x < h_min)
                h_min = s_x; //segment h-pos is leftmost, keep track
            if(s_y < v_min)
                v_min = s_y; //segment v-pos is uppermost, keep track
            if(s_y > v_max)
                v_max = s_y; //segment v-pos is lowermost, keep track
        }
        if(e_x >= left && e_x <= right) {
            //segment's right edge is withing the bounding box
            if(e_x > h_max)
                h_max = e_x; //segment h-pos is rightmost, keep track
            if(s_y < v_min)
                v_min = s_y; //segment v-pos is uppermost, keep track
            if(s_y > v_max)
                v_max = s_y; //segment v-pos is lowermost, keep track
        }
    }

    //find top and bottom
    BOOST_FOREACH(ColourSegment seg, v_segments) {
        //check start
        int s_x = seg.getStart().x,
            s_y = seg.getStart().y,
            e_x = seg.getEnd().x,
            e_y = seg.getEnd().y;
        if(s_x >= left && s_x <= right) {
            //segment's left edge is withing the bounding box
            if(s_y < v_min)
                v_min = s_y; //segment is bottommost, keep track
            else if(s_y > v_max)
                v_max = s_y; //segment is uppermost, keep track
        }
        if(e_x >= left && e_x <= right) {
            //segment's left edge is withing the bounding box
            if(e_y < v_min)
                v_min = e_y; //segment is bottommost, keep track
            else if(e_y > v_max)
                v_max = e_y; //segment is uppermost, keep track
        }
    }

    return Quad(h_min, v_min, h_max, v_max);
}

