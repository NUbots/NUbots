#ifndef RANSACLINE_H
#define RANSACLINE_H

#include "Tools/Math/Line.h"
#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/groundpoint.h"

template<typename T>
class RANSACLine : public Line
{
public:
    RANSACLine() {}

    bool regenerate(const vector<T>& pts) {
        if(pts.size() == minPointsForFit()) {
            setLineFromPoints(pts.at(0), pts.at(1));
            return true;
        }
        else {
            return false;
        }
    }

    inline size_t minPointsForFit() const {return 2;}

    double calculateError(T p) const { return getLinePointDistance(p); }
};


template<>
class RANSACLine<GroundPoint> : public Line
{
public:
    RANSACLine() {}

    bool regenerate(const vector<GroundPoint> &pts) {
        if(pts.size() == minPointsForFit()) {
            setLineFromPoints(pts.at(0).ground, pts.at(1).ground);
            return true;
        }
        else {
            return false;
        }
    }

    inline size_t minPointsForFit() const { return 3; }

    double calculateError(GroundPoint p) const { return getLinePointDistance(p.ground); }
};

#endif // RANSACLINE_H
