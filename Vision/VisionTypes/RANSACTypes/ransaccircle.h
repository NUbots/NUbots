#ifndef RANSACCIRCLE_H
#define RANSACCIRCLE_H

#include <vector>
#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/groundpoint.h"

using std::vector;

template<typename T>
class RANSACCircle
{
public:
    RANSACCircle();

    bool regenerate(const vector<T> &pts);

    unsigned int minPointsForFit() const {return 3;}

    double calculateError(T p) const;

    double getRadius() const {return m_radius;}
    T getCentre() const {return m_centre;}

private:
    bool constructFromPoints(T p1, T p2, T p3, double tolerance = 1.0e-6);

private:
    T m_centre;
    double m_radius;
};

template<>
class RANSACCircle<GroundPoint>
{
public:
    RANSACCircle();

    bool regenerate(const vector<GroundPoint> &pts);

    unsigned int minPointsForFit() const {return 3;}

    double calculateError(GroundPoint p) const;

    double getRadius() const {return m_radius;}
    GroundPoint getCentre() const {return m_centre;}

private:
    bool constructFromPoints(Point p1, Point p2, Point p3, double tolerance = 1.0e-6);

private:
    GroundPoint m_centre;
    double m_radius;
};

#include "ransaccircle.template"

#endif // RANSACCIRCLE_H
