#include "quad.h"
#include <cmath>
#include "Tools/Math/Line.h"



Quad::Quad() : bl(0,0), br(0,0), tl(0,0), tr(0,0)
{
}

Quad::Quad(const Quad& other)
{
    set(other.bl, other.tl, other.tr, other.br);
}

Quad::Quad(Vector2<double> bottom_left, Vector2<double> top_left, Vector2<double> top_right, Vector2<double> bottom_right)
{
    set(bottom_left, top_left, top_right, bottom_right);
}

Quad::Quad(int left, int top, int right, int bottom)
{
    set(left, top, right, bottom);
}

void Quad::set(int left, int top, int right, int bottom)
{
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

Vector2<double> Quad::getCentre() const
{
    return (bl + tl + tr + br)*0.25;
}

double Quad::getAverageWidth() const
{
    return 0.5*((br - bl).abs() + (tr - tl).abs()) + 1;
}

double Quad::getAverageHeight() const
{
    return 0.5*((br - tr).abs() + (bl - tl).abs()) + 1;
}

double Quad::area() const
{
    Line diag(bl, tr);
    return (bl - tr).abs()* (diag.getLinePointDistance(br) + diag.getLinePointDistance(tl) );
}

double Quad::aspectRatio() const
{
    return ( (br - tr).abs() + (bl - tl).abs() + 2 ) / ( (br - bl).abs() + (tr - tl).abs() + 2 );
}

bool Quad::overlapsHorizontally(const Quad &other) const
{
    //rough for now
    double far_right = std::max(tr.x, br.x),
           far_left = std::min(tl.x, bl.x),
           o_far_right = std::max(other.tr.x, other.br.x),
           o_far_left = std::min(other.tl.x, other.bl.x);

    return ! (far_right < o_far_left || o_far_right < far_left);
}

/// @brief Stream insertion operator for a single Quad.
/// @relates Quad
std::ostream& operator<< (std::ostream& output, const Quad& q)
{
    output << q.bl << " " << q.tl << " " << q.tr << " " << q.br;
    return output;
}

/// @brief Stream insertion operator for a std::vector of Quads.
///  @relates Quad
std::ostream& operator<< (std::ostream& output, const std::vector<Quad>& q)
{
    output << "[";
    for (size_t i=0; i<q.size(); i++)
        output << q[i] << ", ";
    output << "]";
    return output;
}
