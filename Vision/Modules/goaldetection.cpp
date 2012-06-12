#include "goaldetection.h"
#include "Vision/visionconstants.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/VisionFieldObjects/beacon.h"

#include <limits>

void GoalDetection::detectGoals()
{    
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    NUImage img = vbb->getOriginalImage();
    const LookUpTable& lut = vbb->getLUT();

    vector<Quad> blue_candidates, yellow_candidates, blue_posts, yellow_posts, blue_beacons, yellow_beacons, unknown_beacons;

    detectGoal(ClassIndex::blue, &blue_candidates);
    detectGoal(ClassIndex::yellow, &yellow_candidates);

    // SPLIT INTO OBJECTS
    splitIntoObjects(&blue_candidates, &yellow_candidates, &blue_posts, &yellow_posts, &blue_beacons, &yellow_beacons, &unknown_beacons);

    // WIDTH CHECK
    widthCheck(&blue_posts);
    widthCheck(&yellow_posts);

    // RATIO CHECK
    ratioCheck(&yellow_posts);
    ratioCheck(&yellow_beacons);
    ratioCheck(&blue_posts);
    ratioCheck(&blue_beacons);
    ratioCheck(&unknown_beacons);

    // OVERLAP CHECK
    overlapCheck(&blue_posts);
    overlapCheck(&yellow_posts);

    // DENSITY CHECK

    // yellow
    DensityCheck(true, &yellow_posts, &img, &lut, VisionConstants::GOAL_MIN_PERCENT_YELLOW);
    DensityCheck(true, &yellow_beacons, &img, &lut, VisionConstants::BEACON_MIN_PERCENT_YELLOW);
    DensityCheck(true, &unknown_beacons, &img, &lut, VisionConstants::BEACON_MIN_PERCENT_YELLOW);

    // blue
    DensityCheck(false, &blue_posts, &img, &lut, VisionConstants::GOAL_MIN_PERCENT_BLUE);
    DensityCheck(false, &blue_beacons, &img, &lut, VisionConstants::BEACON_MIN_PERCENT_BLUE);
    DensityCheck(false, &unknown_beacons, &img, &lut, VisionConstants::BEACON_MIN_PERCENT_BLUE);

    
    //ADD TO BLACKBOARD
    // BLUE BEACONS
    for (unsigned int i = 0; i < blue_beacons.size(); i++) {
        Beacon beacon(Beacon::BlueBeacon, blue_beacons.at(i));
        vbb->addBeacon(beacon);
    }
    // YELLOW BEACONS
    for (unsigned int i = 0; i < yellow_beacons.size(); i++) {
        Beacon beacon(Beacon::YellowBeacon, yellow_beacons.at(i));
        vbb->addBeacon(beacon);
    }
    // AMBIGUOUS BEACONS
    for (unsigned int i = 0; i < unknown_beacons.size(); i++) {
        Beacon beacon(Beacon::UnknownBeacon, unknown_beacons.at(i));
        vbb->addBeacon(beacon);
    }
    
    // BLUE POSTS
    if (blue_posts.size() != 2)
        for (unsigned int i = 0; i < blue_posts.size(); i++) {
            Goal post(Goal::BlueUnknownGoal, blue_posts.at(i));
            vbb->addGoal(post);
        }
    else {
        Quad post1 = blue_posts.at(0),
             post2 = blue_posts.at(1);
        if (post1.getCentre().x < post2.getCentre().x) {
            Goal post_left(Goal::BlueLeftGoal, post1);
            Goal post_right(Goal::BlueRightGoal, post2);
            vbb->addGoal(post_left);
            vbb->addGoal(post_right);
        }
        else {
            Goal post_left(Goal::BlueLeftGoal, post2);
            Goal post_right(Goal::BlueRightGoal, post1);
            vbb->addGoal(post_left);
            vbb->addGoal(post_right);
        }
    }

    // YELLOW POSTS
    if (yellow_posts.size() != 2)
        for (unsigned int i = 0; i < yellow_posts.size(); i++) {
            Goal post(Goal::YellowUnknownGoal, yellow_posts.at(i));
            vbb->addGoal(post);
        }
    else {
        Quad post1 = yellow_posts.at(0),
             post2 = yellow_posts.at(1);
        if (post1.getCentre().x < post2.getCentre().x) {
            Goal post_left(Goal::YellowLeftGoal, post1);
            Goal post_right(Goal::YellowRightGoal, post2);
            vbb->addGoal(post_left);
            vbb->addGoal(post_right);
        }
        else {
            Goal post_left(Goal::YellowLeftGoal, post2);
            Goal post_right(Goal::YellowRightGoal, post1);
            vbb->addGoal(post_left);
            vbb->addGoal(post_right);
        }
    }
}

void GoalDetection::DensityCheck(bool yellow, vector<Quad>* posts, NUImage* img, const LookUpTable* lut, const float PERCENT_REQUIRED)
{
    ClassIndex::Colour colour;
    if (yellow)
        colour = ClassIndex::yellow;
    else
        colour = ClassIndex::blue;

    vector<Quad>::iterator it = posts->begin();
    while (it < posts->end()) {
        Quad candidate = *it;
        int left, right, top, bottom;
        left = candidate.getBottomLeft().x;
        right = candidate.getTopRight().x;
        top = candidate.getTopRight().y;
        bottom = candidate.getBottomLeft().y;

        int count = 0;

        for (int i = left; i < right; i++) {
            for (int j = top; j < bottom; j++) {
                if (ClassIndex::getColourFromIndex(lut->classifyPixel((*img)(i, j))) == colour)
                    count++;
            }
        }
        if (((double)count)/((right-left)*(bottom-top)) < PERCENT_REQUIRED)
            it = posts->erase(it);
        else {
            #if VISION_FIELDOBJECT_VERBOSITY > 1
                debug << "GoalDetection::yellowDensityCheck - goal thrown out on percentage contained yellow" << endl;
            #endif
            it++;
        }
    }
}


void GoalDetection::ratioCheck(vector<Quad>* posts)
{
    const int   HEIGHT_TO_WIDTH_RATIO_LOW = 3,
                HEIGHT_TO_WIDTH_RATIO_HIGH = 15;
    vector<Quad>::iterator it = posts->begin();
    while (it < posts->end()) {
        Quad candidate = *it;
        int height = candidate.getHeight();
        int width = candidate.getWidth();
//        int height = candidate.val[3] - candidate.val[1],
//            width = candidate.val[2] - candidate.val[0];
        if (height/width < HEIGHT_TO_WIDTH_RATIO_LOW || height/width > HEIGHT_TO_WIDTH_RATIO_HIGH)
            it = posts->erase(it);
        else
            it++;
    }
}

void GoalDetection::widthCheck(vector<Quad>* posts)
{
    const int WIDTH_MIN = 2;
    vector<Quad>::iterator it = posts->begin();
    while (it < posts->end()) {
        Quad candidate = *it;
        if (candidate.getWidth() < WIDTH_MIN)
            it = posts->erase(it);
        else
            it++;
    }
}

void GoalDetection::overlapCheck(vector<Quad>* posts)
{
//    for (unsigned int i = 0; i < posts->size(); i++) {
//        Quad left = posts->at(i);
//        for (unsigned int j = i+1; j < posts->size(); j++) {
//            vector<Quad>::iterator it = posts->begin()+j;
//            Quad right = *it;
//            if (left.val[2] >= right.val[0] && left.val[0] <= right.val[2])
//                posts->erase(it);
//        }
//    }
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

void GoalDetection::splitIntoObjects(vector<Quad>* blue_candidates, vector<Quad>* yellow_candidates, vector<Quad>* blue_posts,
                                     vector<Quad>* yellow_posts, vector<Quad>* blue_beacons, vector<Quad>* yellow_beacons,
                                     vector<Quad>* unknown_beacons)
{
    for (unsigned int i = 0; i < blue_candidates->size(); i++) {
        bool is_beacon = false;
        //Quad blue = blue_candidates->at(i);
        cv::Scalar blue = blue_candidates->at(i).getAsScalar();
        if ((int)blue.val[0] < 0) continue;     // FIX THIS
        int b_start = (int)blue.val[0];
        int b_end = (int)blue.val[2];

        for (vector<Quad>::iterator y_it = yellow_candidates->begin(); y_it < yellow_candidates->end(); y_it++) {
            //Quad yellow = *y_it;
            cv::Scalar yellow = y_it->getAsScalar();
            if ((int)yellow.val[0] < 0) continue;
            int y_start = (int)yellow.val[0];
            int y_end = (int)yellow.val[2];
            if ((y_start <= b_start && y_end >= b_end) || (y_start >= b_start && y_start <= b_end) || (y_end >= b_start && y_end <= b_end)) {
                //circle(cvimg, cvPoint(y_start, 10), 1, cv::Scalar(255,255,255), 2, 8, 0);
                int y_top = (int)yellow.val[3];
                int y_bottom = (int)yellow.val[1];
                int b_top = (int)blue.val[3];
                int b_bottom = (int)blue.val[1];
                if (y_top >= b_top && y_bottom <= b_bottom)
                    yellow_beacons->push_back(Quad(min(b_start, y_start), y_bottom, max(b_end, y_end), y_top));
                else if (b_top >= y_top && b_bottom <= y_bottom)
                    blue_beacons->push_back(Quad(min(b_start, y_start), b_bottom, max(b_end, y_end), b_top));
                else
                    unknown_beacons->push_back(Quad(min(b_start, y_start), min(b_bottom, y_bottom), max(b_end, y_end), max(b_top, y_top)));
                is_beacon = true;
                yellow_candidates->erase(y_it);
                break;
            }
        }
        if (!is_beacon)
            blue_posts->push_back(blue_candidates->at(i));  //blue_posts->push_back(blue);
    }
    *yellow_posts = *yellow_candidates;
}


void GoalDetection::detectGoal(ClassIndex::Colour colour, vector<Quad>* candidates)
{
    VisionFieldObject::VFO_ID goal;
    if (colour == ClassIndex::blue)
        goal = VisionFieldObject::GOAL_B;
    else
        goal = VisionFieldObject::GOAL_Y;

    const vector<Transition>& hor_trans = VisionBlackboard::getInstance()->getHorizontalTransitions(goal);
    const vector<Transition>& ver_trans = VisionBlackboard::getInstance()->getVerticalTransitions(goal);

    vector<Transition> start_trans, end_trans;

    // separate into start and end transitions
    for (unsigned int i = 0; i < hor_trans.size(); i++) {
        if (hor_trans.at(i).getAfter() == colour)
            start_trans.push_back(hor_trans.at(i));
        else
            end_trans.push_back(hor_trans.at(i));
    }

    const int MAX_OBJECTS = 8;
    const int BINS = 20;
    const int WIDTH = VisionBlackboard::getInstance()->getImageWidth();
    const int BIN_WIDTH = WIDTH/BINS;
    const int MIN_THRESHOLD = 5;
    const float SDEV_THRESHOLD = 0.75;

    int histogram[2][BINS], peaks[2][MAX_OBJECTS], peak_widths[2][MAX_OBJECTS];
    int merged_peaks[MAX_OBJECTS][2];
    int MAX_WIDTH = 3;

    // REPEAT TWICE; ONCE FOR START TRANSITIONS, ONCE FOR END TRANSITIONS
    for (int repeats = 0; repeats < 2; repeats++) {

        // initialise histograms
        for (int i = 0; i < BINS; i++)
            histogram[repeats][i] = 0;
        for (int i = 0; i < MAX_OBJECTS; i++)
            peak_widths[repeats][i] = 1;

        // fill histogram bins
        if (repeats == 0) {
            for (unsigned int i = 0; i < start_trans.size(); i++)
                for (int j = 0; j < BINS; j++)
                    if (start_trans.at(i).getLocation().x < (j+1)*BIN_WIDTH) {
                        histogram[repeats][j]++;
                        break;
                    }
        }
        else {
            for (unsigned int i = 0; i < end_trans.size(); i++)
                for (int j = 0; j < BINS; j++)
                    if (end_trans.at(i).getLocation().x < (j+1)*BIN_WIDTH) {
                        histogram[repeats][j]++;
                        break;
                    }
        }

        // find MAX_OBJECT peaks
        for (int i = 0; i < MAX_OBJECTS; i++) {
            int max = 0;
            peaks[repeats][i] = 0;
            for (int j = 0; j < BINS; j++)
                if (histogram[repeats][j] > max) {
                    peaks[repeats][i] = j;
                    max = histogram[repeats][j];
                }
            histogram[repeats][peaks[repeats][i]] *= -1;    // prevents from further consideration
        }

        // restore histogram values
        for (int i = 0; i < BINS; i++)
            if (histogram[repeats][i] < 0)
                histogram[repeats][i] *= -1;

        // remove below threshold
        for (int i = 0; i < MAX_OBJECTS; i++)
            if (histogram[repeats][peaks[repeats][i]] < MIN_THRESHOLD)
                peaks[repeats][i] = -1;

        // merge adjacent histogram bins (if both peaks)
        for (int i = 0; i < MAX_OBJECTS; i++) {
            int peak = peaks[repeats][i];
            int span = 0;
            for (int j = i; j < MAX_OBJECTS; j++) {
                if (i == j) continue;
                else if (peak - peaks[repeats][j] <= 1+span && peak - peaks[repeats][j] > 0) {
                    peak_widths[repeats][i] ++;
                    span ++;
                    peaks[repeats][j] = -1;
                }
            }
        }
    }


    // initialise
    for (int i = 0; i < MAX_OBJECTS; i++)
        merged_peaks[i][0] = merged_peaks[i][1] = -1;

    // merge start transitions and end transitions
    for (int i = 0; i < MAX_OBJECTS; i++)
        for (int j = 0; j < MAX_OBJECTS; j++)
            for (int k = 0; k < MAX_WIDTH; k++)
                if (peaks[0][i] == peaks[1][j]-k) {
                    merged_peaks[i][0] = peaks[0][i];
                    merged_peaks[i][1] = peaks[1][j];
                    break;
                }

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
                if (start_trans.at(j).getLocation().x >= start_pos && start_trans.at(j).getLocation().x <= end_pos) {
                    mean +=  start_trans.at(j).getLocation().x;
                    counter++;
                }
            mean /= counter;
            counter = 0;
            for (unsigned int j = 0; j < start_trans.size(); j++)
                if (start_trans.at(j).getLocation().x >= start_pos && start_trans.at(j).getLocation().x <= end_pos) {
                    sdev += pow(static_cast<float>(start_trans.at(j).getLocation().x - mean), 2);
                    counter++;
                }
            sdev = sqrt(sdev/counter);

            for (unsigned int j = 0; j < start_trans.size(); j++) {
                int    x_pos = start_trans.at(j).getLocation().x,
                       y_pos = start_trans.at(j).getLocation().y;
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
                if (end_trans.at(j).getLocation().x >= start_pos && end_trans.at(j).getLocation().x <= end_pos) {
                    mean +=  end_trans.at(j).getLocation().x;
                    counter++;
                }
            mean /= counter;
            counter = 0;
            for (unsigned int j = 0; j < start_trans.size(); j++)
                if (end_trans.at(j).getLocation().x >= start_pos && end_trans.at(j).getLocation().x <= end_pos) {
                    sdev += pow(static_cast<float>(end_trans.at(j).getLocation().x - mean), 2);
                    counter++;
                }
            sdev = sqrt(sdev/counter);

            for (unsigned int j = 0; j < end_trans.size(); j++) {
                int    x_pos = end_trans.at(j).getLocation().x,
                       y_pos = end_trans.at(j).getLocation().y;
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

            for (unsigned int j = 0; j < ver_trans.size(); j++) {
                // extend with vertical segments (increase height)
                int x_pos = ver_trans.at(j).getLocation().x;
                int y_pos = ver_trans.at(j).getLocation().y;
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

