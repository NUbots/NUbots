#include "ransaccircle.h"
#include <cmath>

RANSACCircle::RANSACCircle()
{
}

bool RANSACCircle::regenerate(const vector<Point>& pts)
{
    if(pts.size() == 3) {
        return constructFromPoints(pts[0].screen, pts[1].screen, pts[2].screen, 1.0e-2);
    }
    else {
        return false;
    }
}

double RANSACCircle::calculateError(Point p) const
{
    return std::abs( (p.screen - m_centre.screen).abs() - m_radius);
}
