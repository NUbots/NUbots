/**
*   @name   ObjectDetectionCH
*   @file   objectdetectionch.h
*   @brief  basic object detection by checking breaks in green horizon.
*   @author David Budden
*   @date   22/02/2012
*/

#ifndef OBSTACLEDETECTIONCH_H
#define OBSTACLEDETECTIONCH_H

#include <stdio.h>
#include <iostream>

#include <Vision/basicvisiontypes.h>
#include <Vision/VisionTypes/coloursegment.h>
#include <Vision/VisionTypes/VisionFieldObjects/obstacle.h>

class ObstacleDetectionCH
{
public:
    static std::vector<Obstacle> run();
private:
    void appendEdgesFromSegments(const std::vector<ColourSegment> &segments, std::vector< Point > &pointList);

};

#endif // OBSTACLEDETECTIONCH_H
