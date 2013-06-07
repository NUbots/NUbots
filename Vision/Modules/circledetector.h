#ifndef CIRCLEDETECTOR_H
#define CIRCLEDETECTOR_H

#include <vector>
#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/nupoint.h"
#include "Vision/VisionTypes/VisionFieldObjects/centrecircle.h"

using std::vector;

class CircleDetector
{
public:
    CircleDetector(double tolerance, unsigned int n = 25, unsigned int k = 100, double e = 4.0, unsigned int max_iterations=2);

    void setTolerance(double tolerance);

    virtual bool run(vector<NUPoint> &points, CentreCircle &result);

private:
    unsigned int m_n,
                 m_k,
                 m_max_iterations;
    double m_e,
           m_tolerance;
};

#endif // CIRCLEDETECTOR_H
