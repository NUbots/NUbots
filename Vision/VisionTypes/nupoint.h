#ifndef NUPOINT_H
#define NUPOINT_H

#include "Tools/Math/Vector2.h"
#include "Tools/Math/Vector3.h"

class NUPoint
{
public:
    NUPoint();

    Vector2<double> screenCartesian,
                    screenAngular,
                    groundCartesian;
    Vector3<double>  neckRelativeRadial;

    friend std::istream& operator>>(std::istream& stream, NUPoint& p)
    {
        stream >> p.screenCartesian >> p.screenAngular >> p.groundCartesian >> p.neckRelativeRadial;
        return stream;
    }

    friend std::ostream& operator<<(std::ostream& stream, const NUPoint& p)
    {
        stream << p.screenCartesian << "  " << p.screenAngular << "  " << p.groundCartesian << "  " << p.neckRelativeRadial;
        return stream;
    }
};

#endif // NUPOINT_H
