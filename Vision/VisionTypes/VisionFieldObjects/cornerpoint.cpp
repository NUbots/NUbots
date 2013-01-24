#include "cornerpoint.h"

CornerPoint::CornerPoint(TYPE type, Point screen_location, Vector2<float> relative_location)
{
    m_type = type;
    m_location_pixels = screen_location;
    m_location_angular = relative_location;
}
