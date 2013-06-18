/**
*   @name   ObjectDetectionCH
*   @file   objectdetectionch.cpp
*   @brief  basic object detection by checking breaks in green horizon.
*   @author David Budden
*   @date   22/02/2012
*/

#include "obstacledetectionch.h"
#include "Vision/visionconstants.h"
#include "Vision/VisionTypes/greenhorizon.h"
#include "Vision/visionblackboard.h"
#include "Vision/VisionTools/classificationcolours.h"

#include <boost/foreach.hpp>

//for stat
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

using namespace boost::accumulators;

std::vector<Obstacle> ObstacleDetectionCH::run()
{
    #if VISION_HORIZON_VERBOSITY > 1
        debug << "ObjectDetectionCH::detectObjects() - Begin" << std::endl;
    #endif

    // CONSTANTS
    const int VER_THRESHOLD = 2;                // number of consecutive green pixels required.
    const double OBJECT_THRESHOLD_MULT = 1.5;   // standard deviation multiplier

    // get blackboard instance
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    const LookUpTable& LUT = VisionBlackboard::getInstance()->getLUT();
    const NUImage& img = vbb->getOriginalImage();
    const GreenHorizon& green_horizon = vbb->getGreenHorizon();
    std::vector< Vector2<double> > horizon_points;
    std::vector<Point> obstacle_points;
    std::vector<Obstacle> obstacles;
    int height = img.getHeight();
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
                obstacle_points.push_back(p);
            }
        }
        else {
            // scan from point to bottom of image
            for (int y = p.y; y < height; y++) {
                if ( getColourFromIndex(LUT.classifyPixel(img(p.x,y))) == green ){
                    if (green_count == 1) {
                        green_top = y;
                    }
                    green_count++;
                    // if VER_THRESHOLD green pixels found outside of acceptable range, add point
                    if (green_count == VER_THRESHOLD) {
                        if (green_top > mean_y + OBJECT_THRESHOLD_MULT*std_dev_y + 1) {
                            //only add point if it is outside of minimum distance
                            if (y - green_horizon.getYFromX(p.x) >= VisionConstants::MIN_DISTANCE_FROM_HORIZON) {
                                obstacle_points.push_back(Point(p.x, y));
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
                        obstacle_points.push_back(Point(p.x, y));
                    }
                }
            }
        }
    }

    // update blackboard with obstacle points
    vbb->setObstaclePoints(obstacle_points);

    // find obstacles from these points
    int start = 0;
    int count = 0, bottom = 0;
    bool scanning = false;

    for (unsigned int i = 0; i < obstacle_points.size(); i++) {
        if (!scanning) {
            start = i;
            scanning = true;
            count = 0;
            bottom = 0;
        }
        else {
            if (obstacle_points.at(i).x - obstacle_points.at(i-1).x == static_cast<int>(VisionConstants::VERTICAL_SCANLINE_SPACING) && (i < obstacle_points.size()-1))
            {
                // count while there are consecutive points
                count++;
                if (obstacle_points.at(i).y > bottom)
                    bottom = obstacle_points.at(i).y;
            }
            else {
                // non consecutive found
                if (count > VisionConstants::MIN_CONSECUTIVE_POINTS)
                {
                    // if there are enough then make an obstacle
                    int l = obstacle_points.at(start).x - VisionConstants::VERTICAL_SCANLINE_SPACING;
                    int r = obstacle_points.at(i-1).x + VisionConstants::VERTICAL_SCANLINE_SPACING;

                    int centre = (l + r)*0.5;
                    int width = r - l;
                    int NO_HEIGHT = -1; // DOES NOTHING

                    // generate new obstacle
                    obstacles.push_back(Obstacle(Point(centre, bottom), width, NO_HEIGHT));
                }
                scanning = false;
            }
        }
    }

    // NOW ATTEMPT TO FIND ROBOTS
    std::vector<ColourSegment> cyan_segments = vbb->getAllTransitions(TEAM_CYAN_COLOUR);
    std::vector<ColourSegment> magenta_segments = vbb->getAllTransitions(TEAM_MAGENTA_COLOUR);
    int MIN_COLOUR_THRESHOLD = 20;
    int MAX_OTHER_COLOUR_THRESHOLD = 5;

    for(Obstacle& obst : obstacles)
    {
        int cyan_count = 0;
        int magenta_count = 0;
        double bottom = obst.getLocationPixels().y;
        double left = obst.getLocationPixels().x - obst.getScreenSize().x*0.5;
        double right = obst.getLocationPixels().x + obst.getScreenSize().x*0.5;
        for(const ColourSegment& c : cyan_segments)
        {
            if(c.getEnd().x >= left && c.getEnd().x <= right && c.getEnd().y <= bottom &&
               c.getStart().x >= left && c.getStart().x <= right && c.getStart().y <= bottom)
            {
                cyan_count++;
            }
        }
        for(const ColourSegment& c : magenta_segments)
        {
            if(c.getEnd().x >= left && c.getEnd().x <= right && c.getEnd().y <= bottom &&
               c.getStart().x >= left && c.getStart().x <= right && c.getStart().y <= bottom)
            {
                magenta_count++;
            }
        }

        if(cyan_count >= MIN_COLOUR_THRESHOLD && magenta_count <= MAX_OTHER_COLOUR_THRESHOLD)
        {
            // cyan robot
            obst.m_type = Obstacle::CYAN_ROBOT;
        }
        else if(magenta_count >= MIN_COLOUR_THRESHOLD && cyan_count <= MAX_OTHER_COLOUR_THRESHOLD)
        {
            // magenta robot
            obst.m_type = Obstacle::MAGENTA_ROBOT;
        }
        else
        {
            // unknown obstacle
            obst.m_type = Obstacle::UNKNOWN_OBSTACLE;
        }
    }

    return obstacles;
}

void ObstacleDetectionCH::appendEdgesFromSegments(const std::vector<ColourSegment> &segments, std::vector< Point > &pointList)
{
    std::vector<ColourSegment>::const_iterator it;
    for(it = segments.begin(); it < segments.end(); it++) {
        pointList.push_back(it->getStart());
        pointList.push_back(it->getEnd());
    }
}
