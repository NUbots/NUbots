/**
*   @name   ScanLines
*   @file   scanlines.h
*   @brief  generate horizontal and vertical scanlines.
*   @author David Budden
*   @date   23/03/2012
*/

#ifndef SCANLINES_H
#define SCANLINES_H

#include <stdio.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Vision/visionblackboard.h"
//#include "../VisionTools/classificationcolours.h"

using namespace std;

class ScanLines
{
public:
    /**
    *   @brief  generates the horizontal scanline heights.
    *   This uses the average y positions of the horizon end points and an equal spacing from there
    *   to the top of the image.
    */
    static void generateScanLines();
    
    /**
    *   @brief  classifies each horizontal scanline into blocks of continuous colour.
    */
    static void classifyHorizontalScanLines();
    /**
    *   @brief  classifies each vertical scanline into blocks of continuous colour.
    */
    static void classifyVerticalScanLines();
    
private:
    /**
    *   @brief  classifies a single horizontal scanline.
    */
    static vector<ColourSegment> classifyHorizontalScan(const VisionBlackboard& vbb, const NUImage& img, unsigned int y);
    /**
    *   @brief  classifies a single vertical scanline.
    */
    static vector<ColourSegment> classifyVerticalScan(const VisionBlackboard& vbb, const NUImage& img, const PointType& start);
    
    
};

#endif // SCANLINES_H
