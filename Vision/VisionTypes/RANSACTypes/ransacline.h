#ifndef RANSACLINE_H
#define RANSACLINE_H

#include "Tools/Math/Line.h"
#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/nupoint.h"

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
class RANSACLine<NUPoint> : public Line
{
public:
    RANSACLine() {}

    bool regenerate(const vector<NUPoint> &pts) {
        if(pts.size() == minPointsForFit()) {
            setLineFromPoints(pts.at(0).groundCartesian, pts.at(1).groundCartesian);
            return true;
        }
        else {
            return false;
        }
    }

    inline size_t minPointsForFit() const { return 3; }

    double calculateError(NUPoint p) const { return getLinePointDistance(p.groundCartesian); }
};

#endif // RANSACLINE_H
