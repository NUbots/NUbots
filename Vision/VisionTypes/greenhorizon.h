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

    /**
      * Set the green horizon given a set of hull points. This method performs the necessary
      * interpolation to given a value for any horizontal position along the screen.
      * @param intial_points The scan points that form the hull.
      */
    void set(const vector<PointType>& initial_points);

    //! Returns the y position of the horizon for a given x position.
    int getYFromX(int x) const;
    //! Returns whether the given point is below the horizon.
    bool isBelowHorizon(PointType pt) const;

    //! Returns the original hull points.
    const vector<PointType>& getOriginalPoints() const;
    //! Returns the interpolated points for the entire screen width.
    const vector<PointType>& getInterpolatedPoints() const;
    //! Returns a list of interpolated points with a given spacing.
    vector<PointType> getInterpolatedSubset(unsigned int spacing) const;

private:
    vector<PointType> original_points;      //! @variable The original hull points.
    vector<PointType> interpolated_points;  //! @variable The interpolated points.
};

#endif // GREENHORIZON_H
