/**
*   @name   GreenHorizonCH
*   @file   greenhorizonch.h
*   @brief  calculate green horizon using convex hull method.
*   @author David Budden
*   @date   16/02/2012
*/

#include "greenhorizonch.h"
#include "debug.h"
#include "debugverbosityvision.h"

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
    vector<PointType> horizon_points;
    vector<PointType> temp;
    horizon_points.reserve(VER_SEGMENTS);
    temp.reserve(VER_SEGMENTS);

    const Line& kin_hor = vbb->getKinematicsHorizon();
    int kin_hor_y;

    
    unsigned int position;
    for (unsigned int x = 0; x <= VER_SEGMENTS; x++) {
        position = min(x*width/VER_SEGMENTS, static_cast<unsigned int>(width-1));
        unsigned int green_top = 0;
        unsigned int green_count = 0;


        kin_hor_y = kin_hor.findYFromX(x);
        
        for (int y = kin_hor_y; y < height; y++) {
            if (isPixelGreen(img, position, y)) {
                if (green_count == 0) {
                    green_top = y;
                }
                green_count++;
                // if VER_THRESHOLD green pixels found, add point
                if (green_count == VER_THRESHOLD) {
                    horizon_points.push_back(PointType(position, green_top));
                    break;
                }
            }
            else {
                // not green - reset
                green_count = 0;
            }
            // if no green found, add bottom pixel
            if (y == height-1) {
                horizon_points.push_back(PointType(position, height-1));
            }
        }
    }
    
    // provide blackboard the original set of scan points
    vbb->setHorizonScanPoints(horizon_points);
    
    // statistical filter for green horizon points
    for (unsigned int x = 0; x < VER_SEGMENTS; x++) {
        if (horizon_points.at(x).y < height-1)     // if not at bottom of image
            temp.push_back(horizon_points.at(x));
    }
    Mat mean, std_dev;
    meanStdDev(Mat(temp), mean, std_dev);
    temp.clear();
    
    // copy values into format for convexHull function
    temp.push_back(horizon_points.at(0));
    for (unsigned int x = VER_SEGMENTS-1; x > 0; x--) {
        if (horizon_points.at(x).y > mean.at<double>(1) - UPPER_THRESHOLD_MULT*std_dev.at<double>(1) &&
            horizon_points.at(x).y < mean.at<double>(1) + LOWER_THRESHOLD_MULT*std_dev.at<double>(1)) {
            temp.push_back(horizon_points.at(x));
        }
    }
    
    horizon_points.clear();

    // convex hull
    convexHull(Mat(temp), horizon_points, false, true);

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
    
    // if empty hull
    if (temp.size() <= 2) {
        temp.clear();
        //temp->push_back(PointType(0, height-1));
        //temp->push_back(PointType(width-1, height-1));
        temp.push_back(PointType(0, kin_hor.findYFromX(0)));
        temp.push_back(PointType(width-1, kin_hor.findYFromX(width-1)));
    }
    else {
        // extend to right edge
        if (static_cast<unsigned int>(width-1) > temp.at(temp.size()-1).x + width/VER_SEGMENTS) {
//            temp.push_back(PointType(temp.at(temp.size()-1).x + width/VER_SEGMENTS, height-1));
//            temp.push_back(PointType(width-1, height-1));
            temp.push_back(PointType(width-1, height-1));
        }
        else {
//            temp.push_back(PointType(width-1, height-1));
            temp.push_back(PointType(width-1,temp.at(temp.size()-1).y));
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

    // set hull points
    vbb->setHullPoints(temp);
}


bool GreenHorizonCH::isPixelGreen(const NUImage& img, int x, int y)
{
    const LookUpTable& LUT = VisionBlackboard::getInstance()->getLUT();
    return ClassIndex::getColourFromIndex(LUT.classifyPixel(img(x,y))) == ClassIndex::green;
}
