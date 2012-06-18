#ifndef GOALDETECTION_H
#define GOALDETECTION_H

#include <stdio.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#include "Tools/Math/Line.h"

#include "Vision/visionblackboard.h"
#include "Vision/VisionTools/classificationcolours.h"
#include "Vision/VisionTypes/quad.h"

using namespace std;

class GoalDetection
{
public:
    static void detectGoals();
private:
    static void detectGoal(ClassIndex::Colour colour, vector<Quad>* candidates);
    static void DensityCheck(bool yellow, bool beacon, vector<Quad>* posts, NUImage* img, const LookUpTable* lut, const float PERCENT_REQUIRED);
    static void ratioCheck(vector<Quad>* posts);
    static void widthCheck(vector<Quad>* posts);
    static void splitIntoObjects(vector<Quad>* blue_candidates, vector<Quad>* yellow_candidates, vector<Quad>* blue_posts,
                                         vector<Quad>* yellow_posts, vector<Quad>* blue_beacons, vector<Quad>* yellow_beacons,
                                         vector<Quad>* unknown_beacons);
    static void overlapCheck(vector<Quad>* posts);
};

#endif // GOALDETECTION_H
