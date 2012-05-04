/**
*   @name   HorizonInterpolate
*   @file   horizoninterpolate.h
*   @brief  interpolate green horizon points onto a larger number of vertical segments.
*   @author David Budden
*   @date   21/02/2012
*/

#ifndef HORIZONINTERPOLATE_H
#define HORIZONINTERPOLATE_H

#include <stdio.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Vision/visionblackboard.h"

using namespace std;
using namespace cv;

class HorizonInterpolate
{
public:
    /**
    *   @brief  interpolate green horizon between hull points.
    *   @param  ver_segments The number of required vertical scan segments.
    *   @note   updates blackboard by overwriting original horizon points.
    */
    static void interpolate(const unsigned int ver_segments);
};

#endif // HORIZONINTERPOLATE_H
