#ifndef CIRCLEDETECTOR_H
#define CIRCLEDETECTOR_H

#include <vector>
#include "Tools/Math/Circle.h"

using std::vector;

class CircleDetector
{
public:
    CircleDetector(double tolerance, unsigned int n = 25, unsigned int k = 100, double e = 4.0);

    void setTolerance(double tolerance);

    virtual bool run(vector<Point>& points, Circle& result);

private:
    unsigned int m_n,
                 m_k;
    double m_e,
           m_tolerance;
};

#endif // CIRCLEDETECTOR_H
