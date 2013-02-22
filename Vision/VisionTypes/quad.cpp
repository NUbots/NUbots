#include "quad.h"
#include <cmath>

using namespace std;

Quad::Quad()
{
    set(0,0,0,0);
}

Quad::Quad(const Quad& other)
{
    set(other.bl, other.tl, other.tr, other.br);
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

    bl = Vector2<double>(left, bottom);
    tl = Vector2<double>(left, top);
    tr = Vector2<double>(right, top);
    br = Vector2<double>(right, bottom);
}

void Quad::set(Vector2<double> bottom_left, Vector2<double> top_left, Vector2<double> top_right, Vector2<double> bottom_right)
{
    bl = bottom_left;
    tl = top_left;
    tr = top_right;
    br = bottom_right;
}

Vector2<double> Quad::getBottomCentre() const
{
    return (bl + br)*0.5;
}

Vector2<double> Quad::getCentre() const
{
    return (bl + tl + tr + br)*0.25;
}

int Quad::getBaseWidth() const
{
    return std::abs(br.x - bl.x + 1);
}

int Quad::getTopWidth() const
{
    return std::abs(tr.x - tl.x + 1);
}

double Quad::getAverageWidth() const
{
    return 0.5*(getBaseWidth() + getTopWidth());
}

int Quad::getLeftHeight() const
{
    return std::abs(bl.y - tl.y + 1);
}

int Quad::getRightHeight() const
{
    return std::abs(br.y - tr.y + 1);
}

double Quad::getAverageHeight() const
{
    return 0.5*(getLeftHeight() + getRightHeight());
}

bool Quad::overlapsHorizontally(const Quad &other) const
{
    //rough for now
    double far_right = max(tr.x, br.x),
           far_left = min(tl.x, bl.x),
           o_far_right = max(other.tr.x, other.br.x),
           o_far_left = min(other.tl.x, other.bl.x);

    return ! (far_right < o_far_left || o_far_right < far_left);
}

//void Quad::render(cv::Mat &mat, cv::Scalar colour, bool filled) const
//{
//    cv::Point poly[4] = {cv::Point2f(bl.x, bl.y),
//                         cv::Point2f(tl.x, tl.y),
//                         cv::Point2f(tr.x, tr.y),
//                         cv::Point2f(br.x, br.y)};
//    if(filled) {
//        cv::fillConvexPoly(mat, poly, 4, colour, 4);
//    }
//    else {
//        int num = 4;
//        const cv::Point* p = poly;
//        cv::polylines(mat, &p, &num, 1, true, colour, 2, 4);
//    }
//}
