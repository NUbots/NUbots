#include "quad.h"

Quad::Quad()
{
    set(0,0,0,0);
}

Quad::Quad(const Quad& other)
{
    set(other.m_bottom_left, other.m_top_left, other.m_top_right, other.m_bottom_right);
}

Quad::Quad(int left, int top, int right, int bottom)
{
    set(left, top, right, bottom);
}

Quad::Quad(Vector2<float> bottom_left, Vector2<float> top_left, Vector2<float> top_right, Vector2<float> bottom_right)
{
    set(bottom_left, top_left, top_right, bottom_right);
}

void Quad::set(int left, int top, int right, int bottom)
{
    if(left > right) {
        int temp = left;
        left = right;
        right = temp;
    }
    if(top > bottom) {
        int temp = top;
        top = bottom;
        bottom = temp;
    }

    m_bottom_left = Vector2<float>(left, bottom);
    m_top_left = Vector2<float>(left, top);
    m_top_right = Vector2<float>(right, top);
    m_bottom_right = Vector2<float>(right, bottom);
}

void Quad::set(Vector2<float> bottom_left, Vector2<float> top_left, Vector2<float> top_right, Vector2<float> bottom_right)
{
    m_bottom_left = bottom_left;
    m_top_left = top_left;
    m_top_right = top_right;
    m_bottom_right = bottom_right;
}

Vector2<float> Quad::getBottomCentre() const
{
    return (m_bottom_left + m_bottom_right)*0.5;
}

Vector2<float> Quad::getCentre() const
{
    return (m_bottom_left + m_top_left + m_top_right + m_bottom_right)*0.25;
}

Vector2<float> Quad::getBottomLeft() const
{
    return m_bottom_left;
}

Vector2<float> Quad::getTopRight() const
{
    return m_top_right;
}

int Quad::getBaseWidth() const
{
    return abs(m_bottom_right.x - m_bottom_left.x + 1);
}

int Quad::getTopWidth() const
{
    return abs(m_top_right.x - m_top_left.x + 1);
}

float Quad::getAverageWidth() const
{
    return 0.5*(getBaseWidth() + getTopWidth());
}

int Quad::getLeftHeight() const
{
    return abs(m_bottom_left.y - m_top_left.y + 1);
}

int Quad::getRightHeight() const
{
    return abs(m_bottom_right.y - m_top_right.y + 1);
}

float Quad::getAverageHeight() const
{
    return 0.5*(getLeftHeight() + getRightHeight());
}


