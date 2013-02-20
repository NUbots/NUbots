#include "quad.h"
#include <cmath>

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

Quad::Quad(Vector2<double> bottom_left, Vector2<double> top_left, Vector2<double> top_right, Vector2<double> bottom_right)
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

    m_bottom_left = Vector2<double>(left, bottom);
    m_top_left = Vector2<double>(left, top);
    m_top_right = Vector2<double>(right, top);
    m_bottom_right = Vector2<double>(right, bottom);
}

void Quad::set(Vector2<double> bottom_left, Vector2<double> top_left, Vector2<double> top_right, Vector2<double> bottom_right)
{
    m_bottom_left = bottom_left;
    m_top_left = top_left;
    m_top_right = top_right;
    m_bottom_right = bottom_right;
}

Vector2<double> Quad::getBottomCentre() const
{
    return (m_bottom_left + m_bottom_right)*0.5;
}

Vector2<double> Quad::getCentre() const
{
    return (m_bottom_left + m_top_left + m_top_right + m_bottom_right)*0.25;
}

int Quad::getBaseWidth() const
{
    return std::abs(m_bottom_right.x - m_bottom_left.x + 1);
}

int Quad::getTopWidth() const
{
    return std::abs(m_top_right.x - m_top_left.x + 1);
}

double Quad::getAverageWidth() const
{
    return 0.5*(getBaseWidth() + getTopWidth());
}

int Quad::getLeftHeight() const
{
    return std::abs(m_bottom_left.y - m_top_left.y + 1);
}

int Quad::getRightHeight() const
{
    return std::abs(m_bottom_right.y - m_top_right.y + 1);
}

double Quad::getAverageHeight() const
{
    return 0.5*(getLeftHeight() + getRightHeight());
}

//void Quad::render(cv::Mat &mat, cv::Scalar colour, bool filled) const
//{
//    cv::Point poly[4] = {cv::Point2f(m_bottom_left.x, m_bottom_left.y),
//                         cv::Point2f(m_top_left.x, m_top_left.y),
//                         cv::Point2f(m_top_right.x, m_top_right.y),
//                         cv::Point2f(m_bottom_right.x, m_bottom_right.y)};
//    if(filled) {
//        cv::fillConvexPoly(mat, poly, 4, colour, 4);
//    }
//    else {
//        int num = 4;
//        const cv::Point* p = poly;
//        cv::polylines(mat, &p, &num, 1, true, colour, 2, 4);
//    }
//}
