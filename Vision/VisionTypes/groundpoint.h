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

    friend std::istream& operator>>(std::istream& stream, GroundPoint& g)
    {
        stream >> g.screen >> g.angular >> g.ground >> g.relativeRadial;
        return stream;
    }

    friend std::ostream& operator<<(std::ostream& stream, const GroundPoint& g)
    {
        stream << g.screen << g.angular << g.ground << g.relativeRadial;
        return stream;
    }
};

#endif // POINT_H
