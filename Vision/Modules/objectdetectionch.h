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

#include "Vision/visionblackboard.h"
#include "Vision/VisionTools/classificationcolours.h"



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
};

#endif // OBJECTDETECTIONCH_H
