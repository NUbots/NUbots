/**
 * @file Math/FieldCalculations.cpp
 *
 */
#include "FieldCalculations.h"
#include <cmath>

/*! @brief Calculates the distance between two 2D positions.
 @param x1 x coordinate of the first point.
 @param y1 y coordinate of the first point.
 @param x2 x coordinate of the second point.
 @param y2 y coordinate of the second point.
 */
float DistanceBetweenPoints(float x1, float y1, float x2, float y2)
{
    float diffx = x2 - x1;
    float diffy = y2 - y1;
    return sqrt(diffx*diffx + diffy*diffy);
}

/*! @brief Calculates the angle between two 2D positions.
 @param x1 x coordinate of the first point.
 @param y1 y coordinate of the first point.
 @param x2 x coordinate of the second point.
 @param y2 y coordinate of the second point.
 */
float AngleBetweenPoints(float x1, float y1, float x2, float y2)
{
    float diffx = x2 - x1;
    float diffy = y2 - y1;
    return atan2(diffy,diffx);
}