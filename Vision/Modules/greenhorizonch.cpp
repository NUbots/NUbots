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
        debug << "GreenHorizonCH::calculateHorizon() - Begin" << endl;
    #endif
    // get blackboard instance
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    const NUImage& img = vbb->getOriginalImage();
    int width = img.getWidth(),
        height = img.getHeight();

    //makes this fail-safe in the event of improper parameters
    const int SPACING = max(VisionConstants::GREEN_HORIZON_SCAN_SPACING, 1U);
    
    // variable declarations    
    vector<Point> horizon_points,
                  thrown_points;

    const Horizon& kin_hor = vbb->getKinematicsHorizon();
#if VISION_HORIZON_VERBOSITY > 2
    debug<<"GreenHorizonCH::calculateHorizon(): kinematics horizon line eq: "<<kin_hor.getA()<<"x + "<< kin_hor.getB()<< "y = "<<kin_hor.getC()<<endl;
#endif
    int kin_hor_y;

#if VISION_HORIZON_VERBOSITY > 2
    debug << "GreenHorizonCH::calculateHorizon() - Starting" << endl;
    debug << "GreenHorizonCH::calculateHorizon() - (if seg fault occurs camera or camera cable may be faulty)" << endl;
#endif

    for (int x = 0; x < width; x+=SPACING) {

        unsigned int green_top = 0;
        unsigned int green_count = 0;


        kin_hor_y = kin_hor.findYFromX(x);
        //clamp green horizon values
        kin_hor_y = max(0, kin_hor_y);
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
    DataWrapper::getInstance()->debugPublish(DBID_GREENHORIZON_SCANS, horizon_points);

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
        //changing - I don't see any reason to remove pts that are too low if we are
        //           doing a convex hull anyway.
//        if (p->y < mean_y - VisionConstants::GREEN_HORIZON_UPPER_THRESHOLD_MULT*std_dev_y ||
//            p->y > mean_y + VisionConstants::GREEN_HORIZON_LOWER_THRESHOLD_MULT*std_dev_y) {
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
    debug << "GreenHorizonCH::calculateHorizon() - Statistical filter done" << endl;
#endif

    horizon_points = upperConvexHull(horizon_points);

#if VISION_HORIZON_VERBOSITY > 2
    debug << "GreenHorizonCH::calculateHorizon() - Convex hull done" << endl;
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
        int n = P.size(),
            k = 0;
        vector<Point> H(n);

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
