#ifndef GREENHORIZON_H
#define GREENHORIZON_H

#include <vector>

#include "Vision/basicvisiontypes.h"

using std::vector;

class GreenHorizon
{
public:
    GreenHorizon();
    GreenHorizon(const vector<PointType>& initial_points);

    void set(const vector<PointType>& initial_points);

    int getYFromX(int x) const;
    bool isBelowHorizon(PointType pt) const;

    const vector<PointType>& getOriginalPoints() const;
    const vector<PointType>& getInterpolatedPoints() const;
    vector<PointType> getInterpolatedSubset(unsigned int spacing) const;

private:
    vector<PointType> original_points;
    vector<PointType> interpolated_points;
};

#endif // GREENHORIZON_H
