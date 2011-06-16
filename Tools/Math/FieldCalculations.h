/**
 * @file Math/FieldCalculations.h
 *
 */

#ifndef FIELD_CALCULATIONS_H
#define FIELD_CALCULATIONS_H
#include "Tools/Math/Vector2.h"

float DistanceBetweenPoints(float x1, float y1, float x2, float y2);

float AngleBetweenPoints(float x1, float y1, float x2, float y2);

/*! @brief Calculates the distance between two 2D positions.
 @param p1 first 2D point.
 @param p2 second 2D Point.
 */
inline float DistanceBetweenPoints(Vector2<float> p1, Vector2<float> p2)
{
    return DistanceBetweenPoints(p1.x, p1.y, p2.x, p2.y);
}

/*! @brief Calculates the angle between two 2D positions.
 @param p1 first 2D point.
 @param p2 second 2D Point.
 */
inline float AngleBetweenPoints(Vector2<float> p1, Vector2<float> p2)
{
    return AngleBetweenPoints(p1.x, p1.y, p2.x, p2.y);
}

#endif //FIELD_CALCULATIONS_H
