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
        debug << "ObjectDetectionCH::detectObjects() - Begin" << endl;
    #endif
    // get blackboard instance
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    const NUImage& img = vbb->getOriginalImage();
    unsigned int height = img.getHeight();
    const GreenHorizon& green_horizon = vbb->getGreenHorizon();
    vector< Vector2<double> > horizon_points;
    vector<Point> object_points;
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
    for (unsigned int x = 0; x < horizon_points.size(); x++) {
        unsigned int green_top = 0;
        unsigned int green_count = 0;

        // if bottom of image, assume object
        if (static_cast<unsigned int>(horizon_points.at(x).y) == height-1) {
            object_points.push_back(Point(horizon_points.at(x).x, height-1));
        }
        else {
            // scan from point to bottom of image
            for (unsigned int y = horizon_points.at(x).y; y < height; y++) {
                if (isPixelGreen(img, horizon_points.at(x).x, y)){
                    if (green_count == 1) {
                        green_top = y;
                    }
                    green_count++;
                    // if VER_THRESHOLD green pixels found outside of acceptable range, add point
                    if (green_count == VER_THRESHOLD) {
                        if (green_top > mean_y + OBJECT_THRESHOLD_MULT*std_dev_y + 1) {
                            //cout << "OBJECT: (" << horizon_points->at(x).x << ", " << y << ")" << endl;
                            object_points.push_back(Point(horizon_points.at(x).x, y));
                        }
                        break;
                    }
                }
                // not green - reset
                else {
                    green_count = 0;
                }

                // if bottom reached without green, add bottom point
                if (y == height - 1) {
                    object_points.push_back(Point(horizon_points.at(x).x, y));
                }
            }
        }
    }

    // ignore transitions very close to horizon
    vector<Point>::iterator it;
    it = object_points.begin();
    while (it < object_points.end()) {
//        cout << it->y - green_horizon.getYFromX(it->x) << endl;
        if (it->screen.y - green_horizon.getYFromX(it->screen.x) < VisionConstants::MIN_DISTANCE_FROM_HORIZON) {
            it = object_points.erase(it);
        }
        else {
            it++;
        }
    }

    // update blackboard with object points
    int start = 0, count = 0, bottom = 0;
    bool scanning = false;

    for (unsigned int i = 0; i < object_points.size(); i++) {
        if (!scanning) {
            start = i;
            scanning = true;
            count = 0;
            bottom = 0;
        }
        else {
            if (object_points.at(i).screen.x - object_points.at(i-1).screen.x == static_cast<int>(VisionConstants::VERTICAL_SCANLINE_SPACING) && (i < object_points.size()-1)) {
                count++;
                if (object_points.at(i).screen.y > bottom)
                    bottom = object_points.at(i).screen.y;
            }
            else {
                if (count > VisionConstants::MIN_CONSECUTIVE_POINTS) {
                    int centre = (object_points.at(i-1).screen.x + object_points.at(start).screen.x)*0.5;
                    int width = object_points.at(i-1).screen.x - object_points.at(start).screen.x;
                    int NO_HEIGHT = -1; // DOES NOTHING

                    // push to blackboard
                    Obstacle newObstacle(Point(centre, bottom), width, NO_HEIGHT);
                    vbb->addObstacle(newObstacle);
                }
                scanning = false;
            }
        }
    }


    vbb->setObstaclePoints(object_points);
}


bool ObjectDetectionCH::isPixelGreen(const NUImage& img, int x, int y)
{
    const LookUpTable& LUT = VisionBlackboard::getInstance()->getLUT();
    return getColourFromIndex(LUT.classifyPixel(img(x,y))) == green;
}
