/**
*   @name   GreenHorizonCH
*   @file   greenhorizonch.h
*   @brief  calculate green horizon using convex hull method.
*   @author David Budden
*   @date   16/02/2012
*/

#ifndef GREENHORIZONCH_H
#define GREENHORIZONCH_H

#include <stdio.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Tools/Math/Line.h"

#include "Vision/visionblackboard.h"
#include "Vision/VisionTools/classificationcolours.h"

using namespace std;

class GreenHorizonCH
{
public:
    /**
    *   @brief  calculate green horzion.    
    *   @note   updates blackboard with horizon points.
    */
    static void calculateHorizon();
private:
    /**
    *   @brief  determine whether pixel is green.
    *   @param  img The original image.
    *   @param  x The pixel x coordinate.
    *   @param  y The pixel y coordinate.
    */
    static bool isPixelGreen(const NUImage& img, int x, int y);
    static void convertPointTypes(const vector<cv::Point2i>& cvpoints, vector<PointType>& ourpoints);

    //! CONSTANTS
    //static const unsigned int VER_SEGMENTS = 30;            //! @variable number of vertical scan segments.
    //static const unsigned int VER_THRESHOLD = 5;            //! @variable number of consecutive green pixels required.
    //static const unsigned int UPPER_THRESHOLD_MULT = 2.5;     //! @variable upper standard deviation multiplier
    //static const unsigned int LOWER_THRESHOLD_MULT = 1;     //! @variable lower standard deviation multiplier
};

#endif // GREENHORIZONCH_H
