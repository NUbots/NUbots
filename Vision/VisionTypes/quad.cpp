#include "quad.h"

Quad::Quad()
{
    m_left = m_right = m_bottom = m_top = 0;
    recalculate();
}

Quad::Quad(const Quad& other)
{
    m_left = other.m_left;
    m_right = other.m_right;
    m_bottom = other.m_bottom;
    m_top = other.m_top;
    recalculate();
}

Quad::Quad(int left, int top, int right, int bottom)
{
    m_left = left;
    m_right = right;
    m_bottom = bottom;
    m_top = top;
    recalculate();
}

Vector2<int> Quad::getBottomCentre() const
{
    return m_bottom_centre;
}

Vector2<int> Quad::getCentre() const
{
    return m_centre;
}

Vector2<int> Quad::getBottomLeft() const
{
    return m_bottom_left;
}

Vector2<int> Quad::getTopRight() const
{
    return m_top_right;
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

void Quad::recalculate()
{
    m_centre = Vector2<int>((m_left + m_right)*0.5, (m_bottom + m_top)*0.5);
    m_bottom_centre = Vector2<int>((m_left + m_right)*0.5, std::max(m_bottom, m_top));
    m_bottom_left = Vector2<int>(std::min(m_left, m_right), std::max(m_bottom, m_top));
    m_top_right = Vector2<int>(std::max(m_left, m_right), std::min(m_bottom, m_top));
}
