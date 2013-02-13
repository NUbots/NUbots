#include "Circle.h"
#include <cmath>

Circle::Circle()
{
}

bool Circle::constructFromPoints(Point p1, Point p2, Point p3, double tolerance)
{
    Point ab = p1 - p2,
          bc = p2 - p3;
    double det = ab.x*bc.y-bc.x*ab.y;

    if (std::abs(det) < tolerance) {
        return false;
    }

    double b_len_sqr = p2.squareAbs();

    double ab_norm = (p1.squareAbs() - b_len_sqr)/2.0;
    double bc_norm = (b_len_sqr - p3.squareAbs())/2.0;

    det = 1/det;
    m_centre.x = (ab_norm*(bc.y)-bc_norm*(ab.y))*det;
    m_centre.y = ((ab.x)*bc_norm-(bc.x)*ab_norm)*det;

    m_radius = (m_centre - p1).abs();
    return true;
}
