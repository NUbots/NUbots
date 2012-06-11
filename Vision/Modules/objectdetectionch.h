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
    *   @param  pixel The pixel to test.
    *   @note   STUB - CURRENTLY USES YCC THRESHOLDS (replace with LUT method).
    */
    static bool isPixelGreen(const NUImage& img, int x, int y);
    static void convertPointTypes(const vector<cv::Point2i>& cvpoints, vector<PointType>& ourpoints);
    static void convertPointTypes(const vector<PointType> &ourpoints, vector<cv::Point2i>& cvpoints);

    //! CONSTANTS
    static const unsigned int VER_THRESHOLD = 2;                //! @variable number of consecutive green pixels required.
    static const unsigned int OBJECT_THRESHOLD_MULT = 1.5;      //! @variable lower standard deviation multiplier
};

#endif // OBJECTDETECTIONCH_H
