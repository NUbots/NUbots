#ifndef CORNERPOINT_H
#define CORNERPOINT_H

#include "visionfieldobject.h"

class CornerPoint : public VisionFieldObject
{
public:
    enum TYPE {
        L,
        T,
        CROSS
    };

public:
    CornerPoint(TYPE type, Point screen_location, Vector2<float> relative_location);

private:
    TYPE m_type;
};

#endif // CORNERPOINT_H
