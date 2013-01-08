#include "transitionhistogramming1d.h"
#include "Vision/visionblackboard.h"
#include "Vision/visionconstants.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/VisionFieldObjects/beacon.h"

#include <limits>

#include <boost/foreach.hpp>

void TransitionHistogramming1D::detectGoals()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    NUImage img = vbb->getOriginalImage();
    const LookUpTable& lut = vbb->getLUT();

    vector<Quad> yellow_posts;

    detectGoal(&yellow_posts);

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

void TransitionHistogramming1D::detectGoal(vector<Quad>* candidates)
{
    const vector<ColourSegment>& h_segments = VisionBlackboard::getInstance()->getHorizontalTransitions(VisionFieldObject::GOAL_Y_COLOUR);
    const vector<ColourSegment>& v_segments = VisionBlackboard::getInstance()->getVerticalTransitions(VisionFieldObject::GOAL_Y_COLOUR);

    //vector<PointType> start_trans, end_trans, vert_trans;
    //vector<int> start_lengths, end_lengths;

    const int MAX_PEAKS = 8;
    const int BINS = 20;
    const int WIDTH = VisionBlackboard::getInstance()->getImageWidth();
    const int BIN_WIDTH = WIDTH/BINS;
    const int MIN_THRESHOLD = 1;
    const float SDEV_THRESHOLD = 0.75;

    //int histogram[2][BINS], peaks[2][MAX_OBJECTS], peak_widths[2][MAX_OBJECTS];
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

    // pair start transitions and end transitions
    //COME BACK HERE

    // Calculate bounding boxes for posts
    for (int i = 0; i < MAX_OBJECTS; i++) {
        if (merged_peaks[i][0] >= 0 && merged_peaks[i][1] >= 0) {
            // find bounding box
            int    start_pos = std::max(0, merged_peaks[i][0] * BIN_WIDTH),
                   end_pos = min(WIDTH, (merged_peaks[i][1] + 1) * BIN_WIDTH),
                   start_min = std::numeric_limits<int>::max(),
                   end_max = 0,
                   bot_min = std::numeric_limits<int>::max(),
                   top_max = 0;

            // FIND LEFT EDGE
            int mean = 0, sdev = 0, counter = 0;

            for (unsigned int j = 0; j < start_trans.size(); j++)
                if (start_trans.at(j).x >= start_pos && start_trans.at(j).x <= end_pos) {
                    mean +=  start_trans.at(j).x;
                    counter++;
                }
            mean /= counter;
            counter = 0;
            for (unsigned int j = 0; j < start_trans.size(); j++)
                if (start_trans.at(j).x >= start_pos && start_trans.at(j).x <= end_pos) {
                    sdev += pow(static_cast<float>(start_trans.at(j).x - mean), 2);
                    counter++;
                }
            sdev = sqrt(sdev/counter);

            for (unsigned int j = 0; j < start_trans.size(); j++) {
                int    x_pos = start_trans.at(j).x,
                       y_pos = start_trans.at(j).y;
                if (x_pos >= start_pos && x_pos <= end_pos) {
                    if(x_pos < start_min && x_pos >= mean - SDEV_THRESHOLD*sdev) {
                        start_min = x_pos;
                    }
                    if(y_pos < bot_min)
                        bot_min = y_pos;
                    else if (y_pos > top_max)
                        top_max = y_pos;
                }

            }
            mean = sdev = counter = 0;

            // FIND RIGHT EDGE
            for (unsigned int j = 0; j < end_trans.size(); j++)
                if (end_trans.at(j).x >= start_pos && end_trans.at(j).x <= end_pos) {
                    mean +=  end_trans.at(j).x;
                    counter++;
                }
            mean /= counter;
            counter = 0;
            for (unsigned int j = 0; j < start_trans.size(); j++)
                if (end_trans.at(j).x >= start_pos && end_trans.at(j).x <= end_pos) {
                    sdev += pow(static_cast<float>(end_trans.at(j).x - mean), 2);
                    counter++;
                }
            sdev = sqrt(sdev/counter);

            for (unsigned int j = 0; j < end_trans.size(); j++) {
                int    x_pos = end_trans.at(j).x,
                       y_pos = end_trans.at(j).y;
                if (x_pos >= start_pos && x_pos <= end_pos) {
                    if (x_pos > end_max && x_pos <= mean + SDEV_THRESHOLD*sdev)
                    end_max = x_pos;
                    if(y_pos < bot_min)
                        bot_min = y_pos;
                    else if (y_pos > top_max)
                        top_max = y_pos;
                }
            }

            bool contains_vertical = false;

            //unsigned int    vert_max = 0,
            //                vert_min = std::numeric_limits<int>::max();

            for (unsigned int j = 0; j < vert_trans.size(); j++) {
                // extend with vertical segments (increase height)
                int x_pos = vert_trans.at(j).x;
                int y_pos = vert_trans.at(j).y;
                if (x_pos >= start_min && x_pos <= end_max) {
                    contains_vertical = true;
                    if (y_pos < bot_min)
                        bot_min = y_pos;
                    else if (y_pos > top_max)
                        top_max = y_pos;
                //if (x_pos < vert_min) {
                //    vert_min = x_pos;
                //    cout << "ln356:" << x_pos;
                //}
                //else if (x_pos > vert_max)
                //    vert_max = x_pos;
                }
            }

            // force width to be no greater than that given by vertical segments (inc. SLACK)
//            unsigned int SLACK = 2;
//            if (vert_min - SLACK > start_min)   //THIS SHOULDN'T HAPPEN WHEN THERE ARE NO VERTICAL TRANSITIONS, BUT IT DOES
//                start_min = vert_min - SLACK;
//            if (vert_max + SLACK < end_max)
//                end_max = vert_max + SLACK;

            // throw out if no vertical segments contained
            //if (contains_vertical)

            candidates->push_back(Quad(start_min, bot_min, end_max, top_max));
                //cout << start_min << " " << bot_min << " " << end_max << " " << top_max << endl;
        }
    }
}

Histogram1D TransitionHistogramming1D::mergePeaks(Histogram1D hist, int minimum_size)
{
    hist.mergeAdjacentPeaks(minimum_size);
    return hist;
}
