#ifndef RANSACCIRCLE_H
#define RANSACCIRCLE_H

#include <vector>
#include <cmath>
#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/nupoint.h"

using std::vector;

template<typename T>
class RANSACCircle
{
public:
    RANSACCircle() : m_centre(0,0), m_radius(0) {}

    bool regenerate(const std::vector<T> &pts)
    {
        if(pts.size() == minPointsForFit()) {
            return constructFromPoints(pts[0], pts[1], pts[2], 1.0e-2);
        }
        else {
            return false;
        }
    }

    unsigned int minPointsForFit() const {return 3;}

    double calculateError(T p) const
    {
        return std::abs( (p.screen - m_centre.screen).abs() - m_radius);
    }

    double getRadius() const {return m_radius;}
    T getCentre() const {return m_centre;}

private:
    bool constructFromPoints(T p1, T p2, T p3, double tolerance = 1.0e-6)
    {
        T ab = p1 - p2,
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

private:
    T m_centre;
    double m_radius;
};

template<>
class RANSACCircle<NUPoint>
{
public:
    RANSACCircle() : m_radius(0) {}

    bool regenerate(const std::vector<NUPoint> &pts)
    {
        if(pts.size() == minPointsForFit()) {
            return constructFromPoints(pts[0], pts[1], pts[2], 1.0e-2);
        }
        else {
            return false;
        }
    }

    unsigned int minPointsForFit() const {return 3;}

    double calculateError(NUPoint p) const
    {
        return std::abs( (p.groundCartesian - m_centre.groundCartesian).abs() - m_radius);
    }

    double getRadius() const {return m_radius;}
    NUPoint getCentre() const {return m_centre;}

private:
    bool constructFromPoints(NUPoint p1, NUPoint p2, NUPoint p3, double tolerance = 1.0e-6) {
        Point pa = p1.groundCartesian,
              pb = p2.groundCartesian,
              pc = p3.groundCartesian;
        Point ab = pa - pb,
          bc = pb - pc;
        double det = ab.x*bc.y-bc.x*ab.y;

        if (std::abs(det) < tolerance) {
            return false;
        }

        double b_len_sqr = pb.squareAbs();

        double ab_norm = (pa.squareAbs() - b_len_sqr)/2.0;
        double bc_norm = (b_len_sqr - pc.squareAbs())/2.0;

        det = 1/det;
        m_centre.groundCartesian.x = (ab_norm*(bc.y)-bc_norm*(ab.y))*det;
        m_centre.groundCartesian.y = ((ab.x)*bc_norm-(bc.x)*ab_norm)*det;

        m_radius = (m_centre.groundCartesian - pa).abs();
        return true;
    }

private:
    NUPoint m_centre;
    double m_radius;
};

#endif // RANSACCIRCLE_H
