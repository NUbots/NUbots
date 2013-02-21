#ifndef RANSACLINE_H
#define RANSACLINE_H

#include "Tools/Math/Line.h"
#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/groundpoint.h"

template<typename T>
class RANSACLine : public Line
{
public:
    RANSACLine();

    bool regenerate(const vector<T>& pts);

    unsigned int minPointsForFit() const {return 2;}

    double calculateError(T p) const;
};


template<>
class RANSACLine<GroundPoint> : public Line
{
public:
    RANSACLine();

    bool regenerate(const vector<GroundPoint> &pts);

    unsigned int minPointsForFit() const {return 3;}

    double calculateError(GroundPoint p) const;
};

#include "ransacline.template"
#endif // RANSACLINE_H
