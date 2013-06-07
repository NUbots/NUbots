#ifndef NUPOINT_H
#define NUPOINT_H

#include "Tools/Math/Vector2.h"
#include "Tools/Math/Vector3.h"

class NUPoint
{
public:
    NUPoint();

    Vector2<double> screen,
                    angular,
                    ground;
    Vector3<double>  relativeRadial;

    friend std::istream& operator>>(std::istream& stream, NUPoint& p)
    {
        stream >> p.screen >> p.angular >> p.ground >> p.relativeRadial;
        return stream;
    }

    friend std::ostream& operator<<(std::ostream& stream, const NUPoint& p)
    {
        stream << p.screen << p.angular << p.ground << p.relativeRadial;
        return stream;
    }
};

#endif // NUPOINT_H
