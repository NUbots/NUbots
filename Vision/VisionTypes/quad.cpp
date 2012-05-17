#include "quad.h"

Quad::Quad()
{
    m_left = m_right = m_bottom = m_top = 0;
}

Quad::Quad(const Quad& other)
{
    m_left = other.m_left;
    m_right = other.m_right;
    m_bottom = other.m_bottom;
    m_top = other.m_top;
}

Quad::Quad(int left, int bottom, int right, int top)
{
    m_left = left;
    m_right = right;
    m_bottom = bottom;
    m_top = top;
}

Vector2<int> Quad::getBottomCentre() const
{
    return Vector2<int>((m_right+m_left)*0.5, m_bottom);
}

Vector2<int> Quad::getCentre() const
{
    return Vector2<int>((m_right+m_left)*0.5, (m_bottom+m_top)*0.5);
}

PointType Quad::getBottomLeft() const
{
    return PointType(m_left, m_bottom);
}

PointType Quad::getTopRight() const
{
    return PointType(m_right, m_top);
}

int Quad::getWidth() const
{
    return abs(m_right - m_left);
}

int Quad::getHeight() const
{
    return abs(m_bottom - m_top);
}

cv::Scalar Quad::getAsScalar() const
{
    return cv::Scalar(m_left, m_bottom, m_right, m_top);
}
