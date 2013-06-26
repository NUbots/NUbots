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

using namespace boost::accumulators;

void GreenHorizonCH::calculateHorizon()
{
    #if VISION_HORIZON_VERBOSITY > 1
        debug << "GreenHorizonCH::calculateHorizon() - Begin" << std::endl;
    #endif
    // get blackboard instance
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    const NUImage& img = vbb->getOriginalImage();
    const Horizon& kin_hor = vbb->getKinematicsHorizon();
    int width = img.getWidth();
    int height = img.getHeight();

    //makes this fail-safe in the event of improper parameters
    const int SPACING = std::max(VisionConstants::GREEN_HORIZON_SCAN_SPACING, 1U);
    
    // variable declarations    
    std::vector<Point> horizon_points;
    std::vector<Point> thrown_points;

#if VISION_HORIZON_VERBOSITY > 2
    debug<<"GreenHorizonCH::calculateHorizon(): kinematics horizon line eq: "<<kin_hor.getA()<<"x + "<< kin_hor.getB()<< "y = "<<kin_hor.getC()<<std::endl;
#endif
    int kin_hor_y;

#if VISION_HORIZON_VERBOSITY > 2
    debug << "GreenHorizonCH::calculateHorizon() - Starting" << std::endl;
    debug << "GreenHorizonCH::calculateHorizon() - (if seg fault occurs camera or camera cable may be faulty)" << std::endl;
#endif

    for (int x = 0; x < width; x+=SPACING)
    {
        unsigned int green_top = 0;
        unsigned int green_count = 0;

        kin_hor_y = kin_hor.findYFromX(x);
        //clamp green horizon values
        kin_hor_y = std::max(0, kin_hor_y);
        kin_hor_y = std::min(height-1, kin_hor_y);

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
//            if (y == height-1) {
//                horizon_points.push_back(Point(x, height-1));
//            }
        }
    }

    if(horizon_points.size() < 2) {
        std::cout << "NO GREEN HORIZON FOUND - VERY POOR LUT" << std::endl;
        horizon_points.clear();
        horizon_points.push_back(Point(0, height-1));
        horizon_points.push_back(Point(width-1, height-1));
        vbb->setGreenHullPoints(horizon_points);
        return;
    }
//    if(horizon_points.empty()) {
//        horizon_points.push_back(Point(0, 0));
//        horizon_points.push_back(Point(width-1, 0));
//    }
//    else if(horizon_points.size() == 1) {
//        if(horizon_points.front() == Point(0, 0)) {
//            horizon_points.push_back(Point(width-1, 0));
//        }
//        else if(horizon_points.front() == Point(width-1, 0)) {
//            horizon_points.insert(horizon_points.begin(), 1, Point(0,0));
//        }
//        else {
//            horizon_points.insert(horizon_points.begin(), 1, Point(0, 0));
//            horizon_points.push_back(Point(width-1, 0));
//        }
//    }

#if VISION_HORIZON_VERBOSITY > 2
    debug << "GreenHorizonCH::calculateHorizon() - Green scans done" << std::endl;
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
    debug << "GreenHorizonCH::calculateHorizon() - Statistical filter prep done. Mean: " << mean_y << " Standard Dev: " << std_dev_y << std::endl;
#endif

    std::vector<Point>::iterator p = horizon_points.begin();
    while(p < horizon_points.end()) {
        if (p->y < mean_y - VisionConstants::GREEN_HORIZON_UPPER_THRESHOLD_MULT*std_dev_y) {
            thrown_points.push_back(*p);
            p = horizon_points.erase(p);
        }
        else {
            p++;
        }
    }

    DataWrapper::getInstance()->debugPublish(DBID_GREENHORIZON_THROWN, thrown_points);

#if VISION_HORIZON_VERBOSITY > 2
    debug << "GreenHorizonCH::calculateHorizon() - Statistical filter done" << std::endl;
#endif

    horizon_points = upperConvexHull(horizon_points);

#if VISION_HORIZON_VERBOSITY > 2
    debug << "GreenHorizonCH::calculateHorizon() - Convex hull done" << std::endl;
#endif

    // set hull points
    vbb->setGreenHullPoints(horizon_points);
}


bool GreenHorizonCH::isPixelGreen(const NUImage& img, int x, int y)
{
    const LookUpTable& LUT = VisionBlackboard::getInstance()->getLUT();
    return getColourFromIndex(LUT.classifyPixel(img(x,y))) == green;
}

// Returns a std::list of points on the upper convex hull in clockwise order.
// Note: Assumes the points are sorted in the x direction.
std::vector<Point> GreenHorizonCH::upperConvexHull(const std::vector<Point>& P)
{
        int n = P.size(),
            k = 0;
        std::vector<Point> H(n);

        // Build upper hull
        for (int i = 0; i < n; i++) {
            while (k >= 2 && cross(H[k-2], H[k-1], P[i]) <= 0)
                k--;
            H[k] = P[i];
            k++;
        }

        H.resize(k);
        return H;
}
