/**
*   @name   GreenHorizonCH
*   @file   greenhorizonch.h
*   @brief  calculate green horizon using convex hull method.
*   @author David Budden
*   @date   16/02/2012
*/

#include "greenhorizonch.h"
#include "Kinematics/Horizon.h"
#include "debug.h"
#include "debugverbosityvision.h"
#include "Vision/visionconstants.h"

void GreenHorizonCH::calculateHorizon()
{
    #if VISION_HORIZON_VERBOSITY > 1
        debug << "GreenHorizonCH::calculateHorizon() - Begin" << endl;
    #endif
    // get blackboard instance
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    const NUImage& img = vbb->getOriginalImage();
    int width = img.getWidth(),
        height = img.getHeight();
    #if VISION_HORIZON_VERBOSITY > 2
        debug << "GreenHorizonCH::calculateHorizon() width: " << width << " height: " << height << endl;
    #endif
    
    // variable declarations    
    vector<cv::Point2i> horizon_points;
    vector<cv::Point2i> temp;
    vector<PointType> result;

    const Horizon& kin_hor = vbb->getKinematicsHorizon();
    #if VISION_HORIZON_VERBOSITY > 3
        debug<<"GreenHorizonCH::calculateHorizon(): kinematics horizon line eq: "<<kin_hor.getA()<<"x + "<< kin_hor.getB()<< "y = "<<kin_hor.getC()<<endl;
    #endif
    int kin_hor_y;

    #if VISION_HORIZON_VERBOSITY > 2
        debug << "GreenHorizonCH::calculateHorizon() - Starting" << endl;
    #endif

    for (int x = 0; x < width; x+=VisionConstants::GREEN_HORIZON_SCAN_SPACING) {


        unsigned int green_top = 0;
        unsigned int green_count = 0;


        kin_hor_y = kin_hor.findYFromX(x);
        //clamp green horizon values
        kin_hor_y = max(0,kin_hor_y);
        kin_hor_y = min(height-1, kin_hor_y);


        for (int y = kin_hor_y; y < height; y++) {

            if (isPixelGreen(img, x, y)) {
                if (green_count == 0) {
                    green_top = y;
                }
                green_count++;
                // if VER_THRESHOLD green pixels found, add point
                if (green_count == VisionConstants::GREEN_HORIZON_MIN_GREEN_PIXELS) {
                    horizon_points.push_back(cv::Point2i(x, green_top));
                    break;
                }
            }
            else {
                // not green - reset
                green_count = 0;
            }
            // if no green found, add bottom pixel
            if (y == height-1) {
                horizon_points.push_back(cv::Point2i(x, height-1));
            }
        }

    }
    
    #if VISION_HORIZON_VERBOSITY > 2
        debug << "GreenHorizonCH::calculateHorizon() - Green scans done" << endl;
    #endif
    // provide blackboard the original set of scan points
    convertPointTypes(horizon_points, result);
    vbb->setGreenHorizonScanPoints(result);
    // statistical filter for green horizon points
    for (unsigned int x = 0; x < horizon_points.size(); x++) {
        if (horizon_points.at(x).y < height-1)     // if not at bottom of image
            temp.push_back(horizon_points.at(x));
    }
    cv::Mat mean, std_dev;
    meanStdDev(cv::Mat(temp), mean, std_dev);
    temp.clear();
    
    #if VISION_HORIZON_VERBOSITY > 2
        debug << "GreenHorizonCH::calculateHorizon() - Statistical filter prep done" << endl;
    #endif
    
    // copy values into format for convexHull function
    temp.push_back(horizon_points.at(0));
    for (unsigned int x = horizon_points.size()-1; x > 0; x--) {
        if (horizon_points.at(x).y > mean.at<double>(1) - VisionConstants::GREEN_HORIZON_UPPER_THRESHOLD_MULT*std_dev.at<double>(1) &&
            horizon_points.at(x).y < mean.at<double>(1) + VisionConstants::GREEN_HORIZON_LOWER_THRESHOLD_MULT*std_dev.at<double>(1)) {
            temp.push_back(horizon_points.at(x));
        }
    }
    #if VISION_HORIZON_VERBOSITY > 2
        debug << "GreenHorizonCH::calculateHorizon() - Statistical filter done" << endl;
    #endif
    
    horizon_points.clear();

    // convex hull
    convexHull(cv::Mat(temp), horizon_points, false, true);

    #if VISION_HORIZON_VERBOSITY > 2
        debug << "GreenHorizonCH::calculateHorizon() - Convex hull done" << endl;
    #endif
    
    // get top half (silly ordering)
    temp.clear();
    bool top = false;   // is LHS point found

    for (unsigned int x = 0; x < horizon_points.size(); x++) {
        // until LHS found, add nothing
        if (horizon_points.at(x).x == 0) {
            top = true;
            temp.push_back(horizon_points.at(x));
        }
        // once found, add remaining points
        else if (top) {
            temp.push_back(horizon_points.at(x));
        }
    }
    // add RHS point
    temp.push_back(horizon_points.at(0));
    
    #if VISION_HORIZON_VERBOSITY > 2
        debug << "GreenHorizonCH::calculateHorizon() - Convex hull reordering done" << endl;
    #endif
    
    // if empty hull
    if (temp.size() <= 2) {
        temp.clear();
        //temp->push_back(PointType(0, height-1));
        //temp->push_back(PointType(width-1, height-1));
        int kin_hor_left_y = kin_hor.findYFromX(0),
            kin_hor_right_y = kin_hor.findYFromX(width-1);
        //clamp kinematics horizon values
        kin_hor_left_y = max(0,kin_hor_left_y);
        kin_hor_right_y = max(0,kin_hor_right_y);
        kin_hor_left_y = min(height-1, kin_hor_left_y);
        kin_hor_right_y = min(height-1, kin_hor_right_y);
        //add new points at edge
        temp.push_back(cv::Point2i(0, kin_hor_left_y));
        temp.push_back(cv::Point2i(width-1, kin_hor_right_y));
    }
    else {
        // extend to right edge
        if (static_cast<unsigned int>(width-1) > temp.at(temp.size()-1).x + VisionConstants::GREEN_HORIZON_SCAN_SPACING) {
//            temp.push_back(PointType(temp.at(temp.size()-1).x + width/VER_SEGMENTS, height-1));
//            temp.push_back(PointType(width-1, height-1));
            temp.push_back(cv::Point2i(width-1, height-1));
        }
        else {
//            temp.push_back(PointType(width-1, height-1));
            temp.push_back(cv::Point2i(width-1,temp.at(temp.size()-1).y));
        }

        // extend to left edge
//        if (temp.at(0).y == height-1) {
//            if (temp.at(1).x > width/static_cast<int>(VER_SEGMENTS)) {
//                //temp->insert(1, PointType(0, 0));
//                vector<PointType>::iterator it;
//                it = temp.begin();
//                it++;
//                it = temp.insert (it , PointType(temp.at(1).x - width/VER_SEGMENTS, height-1));
//            }
//        }
    }
    #if VISION_HORIZON_VERBOSITY > 2
        debug << "GreenHorizonCH::calculateHorizon() - Side extension done" << endl;
    #endif
    #if VISION_HORIZON_VERBOSITY > 3
    // set hull points
    for (int l = 0; l<temp.size(); l++){
        debug<<"GreenHorizonCH::calculateHorizon(): temp["<<l<< "] ="<<temp[l].x<< ", "<< temp[l].y <<endl;
    }
    for (int l = 0; l<result.size(); l++){
        debug<<"GreenHorizonCH::calculateHorizon(): result["<<l<< "] ="<<result[l].x<< ", "<< result[l].y <<endl;
    }
    #endif

    convertPointTypes(temp, result);
    vbb->setGreenHullPoints(result);
}


bool GreenHorizonCH::isPixelGreen(const NUImage& img, int x, int y)
{
    const LookUpTable& LUT = VisionBlackboard::getInstance()->getLUT();
    return ClassIndex::getColourFromIndex(LUT.classifyPixel(img(x,y))) == ClassIndex::green;
}

void GreenHorizonCH::convertPointTypes(const vector<cv::Point2i> &cvpoints, vector<PointType> &ourpoints)
{
    ourpoints.clear();
    for(unsigned int i=0; i<cvpoints.size(); i++) {
        ourpoints.push_back(PointType(cvpoints.at(i).x, cvpoints.at(i).y));
    }
}
