/**
*   @name   ObjectDetectionCH
*   @file   objectdetectionch.h
*   @brief  basic object detection by checking breaks in green horizon.
*   @author David Budden
*   @date   22/02/2012
*/

#ifndef OBJECTDETECTIONCH_H
#define OBJECTDETECTIONCH_H

#include <stdio.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Vision/visionblackboard.h"
#include "Vision/VisionTools/classificationcolours.h"

using namespace std;

class ObjectDetectionCH
{
public:
    static void detectObjects();
private:
    /**
    *   @brief  determine whether pixel is green.
    *   @param  img The original image.
    *   @param  x The pixel x coordinate.
    *   @param  y The pixel y coordinate.
    *   @return whether the pixel is green
    */
    static bool isPixelGreen(const NUImage& img, int x, int y);
    /**
    *   @brief  convert cv::Point2i from opencv functions into our point type.
    *   @param  cvpoints a reference to a vector of cv::points to be converted.
    *   @param  outpoints a reference to a vector of PointType to be filled.
    */
    static void convertPointTypes(const vector<cv::Point2i>& cvpoints, vector<Vector2<double> >& ourpoints);
    /**
    *   @brief  convert our point type into cv::Point2i for opencv functions.
    *   @param  outpoints a reference to a vector of PointType to be converted.
    *   @param  cvpoints a reference to a vector of cv::Point2i to be filled.
    */
    static void convertPointTypes(const vector<Vector2<double> > &ourpoints, vector<cv::Point2i>& cvpoints);

    //! CONSTANTS
    static const unsigned int VER_THRESHOLD = 2;                //! @variable number of consecutive green pixels required.
    static const unsigned int OBJECT_THRESHOLD_MULT = 1.5;      //! @variable lower standard deviation multiplier
};

#endif // OBJECTDETECTIONCH_H
