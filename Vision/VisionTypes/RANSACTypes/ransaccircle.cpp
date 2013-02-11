#include "ransaccircle.h"
#include <cmath>

RANSACCircle::RANSACCircle()
{
}

bool RANSACCircle::regenerate(const vector<Point>& pts)
{
    if(pts.size() == 3) {
        return constructFromPoints(pts[0], pts[1], pts[2], 1.0e-2);
    }
    else {
        return false;
    }
}

double RANSACCircle::calculateError(Point p) const
{
    return std::abs( (p - m_centre).abs() - m_radius);
}
