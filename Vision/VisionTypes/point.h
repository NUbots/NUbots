#ifndef POINT_H
#define POINT_H

#include "Tools/Math/Vector2.h"
#include "Tools/Math/Vector3.h"

class Point
{
public:
    Point(double screen_x = 0.0, double screen_y = 0.0);
    Point(Vector2<double> screen_pos);

    Vector2<double> screen,
                    angular,
                    ground;
    Vector3<double>  relativeRadial;
};

#endif // POINT_H
