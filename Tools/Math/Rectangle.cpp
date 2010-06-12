#include "Rectangle.h"
#include "debug.h"
Rectangle::Rectangle()
{
    m_minx = 0;
    m_maxx = 0;
    m_miny = 0;
    m_maxy = 0;
}

Rectangle::Rectangle(const Rectangle& source)
{
    m_minx = source.m_minx;
    m_maxx = source.m_maxx;
    m_miny = source.m_miny;
    m_maxy = source.m_maxy;
}

Rectangle::Rectangle(float minx, float maxx, float miny, float maxy)
{
    m_minx = minx;
    m_maxx = maxx;
    m_miny = miny;
    m_maxy = maxy;
}

void Rectangle::Set(float minx, float maxx, float miny, float maxy)
{
    m_minx = minx;
    m_maxx = maxx;
    m_miny = miny;
    m_maxy = maxy;
}

float Rectangle::CentreX()
{
    return m_minx + Width()/2.0;
}

float Rectangle::CentreY()
{
    return m_miny + Height()/2.0;
}

float Rectangle::Width()
{
    return m_maxx - m_minx;
}

float Rectangle::Height()
{
    return m_maxy - m_miny;
}

float Rectangle::Area()
{
    return Width() * Height();
}

bool Rectangle::PointInside(float x, float y)
{
    // From: http://www.gamedev.net/community/forums/topic.asp?topic_id=483716

    // Distance of point from bottom left corner
    float vpx =  x - m_minx;
    float vpy =  y - m_miny;

    // Vector of bottom side
    float vbx = m_maxx - m_minx;
    float vby = m_miny - m_miny;

    // Vector of left side
    float vlx = m_minx - m_minx;
    float vly = m_maxy - m_miny;

    float dotvpvbx = vpx*vbx + vpy*vby;
    float dotvbvbx = vbx*vbx + vby*vby;
    float dotvpvlx = vpx*vlx + vpy*vly;
    float dotvlvlx = vlx*vlx + vly*vly;

    bool inside = (0.0f < dotvpvbx) && (dotvpvbx < dotvbvbx) && (0.0f < dotvpvlx) && (dotvpvlx < dotvlvlx);
    return inside;
}

float Rectangle::MinX()
{
    return m_minx;
}

float Rectangle::MaxX()
{
    return m_maxx;
}

float Rectangle::MinY()
{
    return m_miny;
}

float Rectangle::MaxY()
{
    return m_maxy;
}
