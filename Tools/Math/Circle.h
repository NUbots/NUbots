#ifndef CIRCLE_H
#define CIRCLE_H

#include "Tools/Math/Vector2.h"

class Circle
{
public:
    Circle();

    bool constructFromPoints(Vector2<double> p1, Vector2<double> p2, Vector2<double> p3, double tolerance = 1.0e-6);

    double getRadius() const {return m_radius;}
    Vector2<double> getCentre() const {return m_centre;}

protected:
    Vector2<double> m_centre;
    double m_radius;
};

#endif // CIRCLE_H
