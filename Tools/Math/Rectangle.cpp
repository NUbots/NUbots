#include "Rectangle.h"

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
