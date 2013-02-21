#ifndef POINT_H
#define POINT_H

#include "Tools/Math/Vector2.h"
#include "Tools/Math/Vector3.h"

class GroundPoint
{
public:
    GroundPoint();

    Vector2<double> screen,
                    angular,
                    ground;
    Vector3<double>  relativeRadial;
};

#endif // POINT_H
