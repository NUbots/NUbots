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

#include <boost/foreach.hpp>

//for stat
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

////for convex hull
//#include <boost/geometry.hpp>
//#include <boost/geometry/geometries/polygon.hpp>
//#include <boost/geometry/geometries/adapted/boost_tuple.hpp>

using namespace boost::accumulators;

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
    vector<Point> horizon_points;

    const Horizon& kin_hor = vbb->getKinematicsHorizon();
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
                    horizon_points.push_back(Point(x, green_top));
                    break;
                }
            }
            else {
                // not green - reset
                green_count = 0;
            }
            // if no green found, add bottom pixel
            if (y == height-1) {
                horizon_points.push_back(Point(x, height-1));
            }
        }
    }
    
    #if VISION_HORIZON_VERBOSITY > 2
        debug << "GreenHorizonCH::calculateHorizon() - Green scans done" << endl;
    #endif
    // provide blackboard the original set of scan points
    vbb->setGreenHorizonScanPoints(horizon_points);

    // statistical filter for green horizon points
    double mean_y, std_dev_y;
    accumulator_set<double, stats<tag::mean, tag::variance> > acc;

    BOOST_FOREACH(Point& p, horizon_points) {
        if (p.y < height-1)     // if not at bottom of image
            acc(p.y);
    }

    mean_y = mean(acc);
    std_dev_y = sqrt(variance(acc));
    
    #if VISION_HORIZON_VERBOSITY > 2
    debug << "GreenHorizonCH::calculateHorizon() - Statistical filter prep done. Mean: " << mean_y << " Standard Dev: " << std_dev_y << endl;
    #endif

    vector<Point>::iterator p = horizon_points.begin();
    while(p < horizon_points.end()) {
        if (p->y < mean_y - VisionConstants::GREEN_HORIZON_UPPER_THRESHOLD_MULT*std_dev_y ||
            p->y > mean_y + VisionConstants::GREEN_HORIZON_LOWER_THRESHOLD_MULT*std_dev_y) {
            p = horizon_points.erase(p);
        }
        else {
            p++;
        }
    }
    #if VISION_HORIZON_VERBOSITY > 2
        debug << "GreenHorizonCH::calculateHorizon() - Statistical filter done" << endl;
    #endif

//    // convex hull
//    typedef boost::tuple<double, double> poly_point;
//    typedef model::polygon<poly_point> polygon;

//    polygon poly, hull;

//    BOOST_FOREACH(Point& p, horizon_points) {
//        append(poly, poly_point(p.x, p.y));
//    }

//    convex_hull(poly, hull);

//    //get points from boost hull
//    horizon_points.clear();
//    BOOST_FOREACH(poly_point p, hull) {
//        horizon_points.push_back(Point(p.x, p.y));
//    }
    horizon_points = upperConvexHull(horizon_points);

    #if VISION_HORIZON_VERBOSITY > 2
        debug << "GreenHorizonCH::calculateHorizon() - Convex hull done" << endl;
    #endif
    
//    // get top half (silly ordering)
//    temp.clear();
//    bool start_found = false;   // is LHS point found

//    for (unsigned int x = 0; x < horizon_points.size(); x++) {
//        // until LHS found, add nothing
//        if (horizon_points.at(x).x == 0) {
//            start_found = true;
//            temp.push_back(horizon_points.at(x));
//        }
//        // once found, add remaining points
//        else if (start_found) {
//            temp.push_back(horizon_points.at(x));
//        }
//    }
//    // add RHS point
//    temp.push_back(horizon_points.at(0));
    
//    #if VISION_HORIZON_VERBOSITY > 2
//        debug << "GreenHorizonCH::calculateHorizon() - Convex hull reordering done" << endl;
//    #endif
    
//    // if empty hull
//    if (temp.size() <= 2) {
//        temp.clear();
//        //temp->push_back(PointType(0, height-1));
//        //temp->push_back(PointType(width-1, height-1));
//        int kin_hor_left_y = kin_hor.findYFromX(0),
//            kin_hor_right_y = kin_hor.findYFromX(width-1);
//        //clamp kinematics horizon values
//        kin_hor_left_y = max(0,kin_hor_left_y);
//        kin_hor_right_y = max(0,kin_hor_right_y);
//        kin_hor_left_y = min(height-1, kin_hor_left_y);
//        kin_hor_right_y = min(height-1, kin_hor_right_y);
//        //add new points at edge

//        temp.push_back(cv::Point2i(0, kin_hor_left_y));
//        temp.push_back(cv::Point2i(width-1, kin_hor_right_y));
////        temp.push_back(cv::Point2i(0, height-1));
////        temp.push_back(cv::Point2i(width-1, height-1));
//    }
//    else {
        // extend to right edge
        if (static_cast<unsigned int>(width-1) > horizon_points.back().x + VisionConstants::GREEN_HORIZON_SCAN_SPACING) {
//            temp.push_back(PointType(temp.at(temp.size()-1).x + width/VER_SEGMENTS, height-1));
//            temp.push_back(PointType(width-1, height-1));
            horizon_points.push_back(Point(width-1, height-1));
        }
        else {
//            temp.push_back(PointType(width-1, height-1));
            horizon_points.push_back(Point(width-1,horizon_points.back().y));
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
//    }
    #if VISION_HORIZON_VERBOSITY > 2
        debug << "GreenHorizonCH::calculateHorizon() - Side extension done" << endl;
    #endif

    // set hull points
    vbb->setGreenHullPoints(horizon_points);
}


bool GreenHorizonCH::isPixelGreen(const NUImage& img, int x, int y)
{
    const LookUpTable& LUT = VisionBlackboard::getInstance()->getLUT();
    return getColourFromIndex(LUT.classifyPixel(img(x,y))) == green;
}

// Returns a list of points on the upper convex hull in clockwise order.
// Note: Assumes the points are sorted in the x direction.
vector<Point> GreenHorizonCH::upperConvexHull(const vector<Point>& P)
{
        int n = P.size(), k = 0;
        vector<Point> H(2*n);

        // Build upper hull
        for (int i = 0; i < n; i++) {
            while (k >= 2 && cross(H[k-2], H[k-1], P[i]) >= 0)
                k--;
            H[k] = P[i];
            k++;
        }

        H.resize(k);
        return H;
}
