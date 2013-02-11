#ifndef CIRCLE_H
#define CIRCLE_H

#include "Tools/Math/Vector2.h"

typedef Vector2<double> Point;

class Circle
{
public:
    Circle();

    bool constructFromPoints(Point p1, Point p2, Point p3, double tolerance = 1.0e-6);

protected:
    Point m_centre;
    double m_radius;
};

#endif // CIRCLE_H
