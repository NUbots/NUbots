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

#include "Tools/Math/Line.h"

#include "Vision/visionblackboard.h"
#include "Vision/VisionTools/classificationcolours.h"



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
    *   @return whether the pixel is green
    */
    static bool isPixelGreen(const NUImage& img, int x, int y);

    // 2D cross product of OA and OB std::vectors, i.e. z-component of their 3D cross product.
    // Returns a positive value, if OAB makes a counter-clockwise turn,
    // negative for clockwise turn, and zero if the points are collinear.
    static double cross(const Vector2<double> &O, const Vector2<double> &A, const Vector2<double> &B)
    {
        return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
    }

    // Returns a std::list of points on the convex hull in counter-clockwise order.
    // Note: the last point in the returned std::list is the same as the first one.
    static std::vector< Vector2<double> > upperConvexHull(const std::vector< Vector2<double> >& points);
    //! CONSTANTS
    //static const unsigned int VER_SEGMENTS = 30;            //! @variable number of vertical scan segments.
    //static const unsigned int VER_THRESHOLD = 5;            //! @variable number of consecutive green pixels required.
    //static const unsigned int UPPER_THRESHOLD_MULT = 2.5;     //! @variable upper standard deviation multiplier
    //static const unsigned int LOWER_THRESHOLD_MULT = 1;     //! @variable lower standard deviation multiplier
};

#endif // GREENHORIZONCH_H
