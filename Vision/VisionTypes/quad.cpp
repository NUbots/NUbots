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

/// @brief Stream insertion operator for a single Quad.
/// @relates Quad
ostream& operator<< (ostream& output, const Quad& q)
{
    output << q.bl << " " << q.tl << " " << q.tr << " " << q.br;
    return output;
}

/// @brief Stream insertion operator for a vector of Quads.
///  @relates Quad
ostream& operator<< (ostream& output, const vector<Quad>& q)
{
    output << "[";
    for (size_t i=0; i<q.size(); i++)
        output << q[i] << ", ";
    output << "]";
    return output;
}
