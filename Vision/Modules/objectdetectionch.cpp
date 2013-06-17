/**
*   @name   ObjectDetectionCH
*   @file   objectdetectionch.cpp
*   @brief  basic object detection by checking breaks in green horizon.
*   @author David Budden
*   @date   22/02/2012
*/

#include "objectdetectionch.h"
#include "Vision/visionconstants.h"
#include "Vision/VisionTypes/greenhorizon.h"

#include <boost/foreach.hpp>

//for stat
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

using namespace boost::accumulators;

void ObjectDetectionCH::detectObjects()
{
    #if VISION_HORIZON_VERBOSITY > 1
        debug << "ObjectDetectionCH::detectObjects() - Begin" << std::endl;
    #endif
    // get blackboard instance
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    const NUImage& img = vbb->getOriginalImage();
    int height = img.getHeight();
    const GreenHorizon& green_horizon = vbb->getGreenHorizon();
    std::vector< Vector2<double> > horizon_points;
    std::vector<Point> object_points;
    double mean_y,
           std_dev_y;

    //get scan points from BB
    horizon_points = green_horizon.getInterpolatedSubset(VisionConstants::VERTICAL_SCANLINE_SPACING);

    //calculate mean and stddev of vertical positions
    accumulator_set<double, stats<tag::mean, tag::variance> > acc;

    BOOST_FOREACH(Vector2<double>& p, horizon_points) {
        acc(p.y);
    }

    mean_y = mean(acc);
    std_dev_y = sqrt(variance(acc));

    // for each point in interpolated list
    for (Point p : horizon_points) {
        int green_top = 0;
        int green_count = 0;

        // if bottom of image, assume object
        if (p.y == height-1) {
            if (p.y - green_horizon.getYFromX(p.x) >= VisionConstants::MIN_DISTANCE_FROM_HORIZON) {
                object_points.push_back(p);
            }
        }
        else {
            // scan from point to bottom of image
            for (int y = p.y; y < height; y++) {
                if (isPixelGreen(img, p.x, y)){
                    if (green_count == 1) {
                        green_top = y;
                    }
                    green_count++;
                    // if VER_THRESHOLD green pixels found outside of acceptable range, add point
                    if (green_count == VER_THRESHOLD) {
                        if (green_top > mean_y + OBJECT_THRESHOLD_MULT*std_dev_y + 1) {
                            //only add point if it is outside of minimum distance
                            if (y - green_horizon.getYFromX(p.x) >= VisionConstants::MIN_DISTANCE_FROM_HORIZON) {
                                object_points.push_back(Point(p.x, y));
                            }
                        }
                        break;
                    }
                }
                else {
                    green_count = 0; // not green - reset
                }

                // if bottom reached without green, add bottom point
                if (y == height - 1) {
                    if (y - green_horizon.getYFromX(p.x) >= VisionConstants::MIN_DISTANCE_FROM_HORIZON) {
                        object_points.push_back(Point(p.x, y));
                    }
                }
            }
        }
    }

    vbb->setObstaclePoints(object_points);

    // update blackboard with object points
    int start = 0;
    int count = 0, bottom = 0;
    bool scanning = false;

    for (unsigned int i = 0; i < object_points.size(); i++) {
        if (!scanning) {
            start = i;
            scanning = true;
            count = 0;
            bottom = 0;
        }
        else {
            if (object_points.at(i).x - object_points.at(i-1).x == static_cast<int>(VisionConstants::VERTICAL_SCANLINE_SPACING) && (i < object_points.size()-1))
            {
                // count while there are consecutive points
                count++;
                if (object_points.at(i).y > bottom)
                    bottom = object_points.at(i).y;
            }
            else {
                // non consecutive found
                if (count > VisionConstants::MIN_CONSECUTIVE_POINTS)
                {
                    // if there are enough then make an obstacle
                    int l = object_points.at(start).x - VisionConstants::VERTICAL_SCANLINE_SPACING;
                    int r = object_points.at(i-1).x + VisionConstants::VERTICAL_SCANLINE_SPACING;

                    int centre = (l + r)*0.5;
                    int width = r - l;
                    int NO_HEIGHT = -1; // DOES NOTHING

                    // push to blackboard
                    Obstacle newObstacle(Point(centre, bottom), width, NO_HEIGHT);
                    vbb->addObstacle(newObstacle);
                }
                scanning = false;
            }
        }
    }
}


bool ObjectDetectionCH::isPixelGreen(const NUImage& img, int x, int y)
{
    const LookUpTable& LUT = VisionBlackboard::getInstance()->getLUT();
    return getColourFromIndex(LUT.classifyPixel(img(x,y))) == green;
}
