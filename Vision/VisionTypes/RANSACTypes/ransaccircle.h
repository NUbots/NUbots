#ifndef RANSACCIRCLE_H
#define RANSACCIRCLE_H

#include <vector>
#include "Tools/Math/Circle.h"

using std::vector;

class RANSACCircle : public Circle
{
public:
    RANSACCircle();

    bool regenerate(const vector<Point> &pts);

    unsigned int minPointsForFit() const {return 3;}

    double calculateError(Point p) const;
};

#endif // RANSACCIRCLE_H
