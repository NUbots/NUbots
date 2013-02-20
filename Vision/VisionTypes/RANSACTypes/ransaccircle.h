#ifndef RANSACCIRCLE_H
#define RANSACCIRCLE_H

#include <vector>
#include "Vision/basicvisiontypes.h"

using std::vector;

class RANSACCircle
{
public:
    RANSACCircle();

    bool regenerate(const vector<Point> &pts);

    unsigned int minPointsForFit() const {return 3;}

    double calculateError(Point p) const;

    double getRadius() const {return m_radius;}
    Point getCentre() const {return m_centre;}

private:
    bool constructFromPoints(Vector2<double> p1, Vector2<double> p2, Vector2<double> p3, double tolerance = 1.0e-6);


private:
    Point m_centre;
    double m_radius;
};

#endif // RANSACCIRCLE_H
