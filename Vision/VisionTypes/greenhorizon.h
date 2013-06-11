#ifndef GREENHORIZON_H
#define GREENHORIZON_H

#include <vector>

#include "Vision/basicvisiontypes.h"

using std::vector;

class GreenHorizon
{
public:
    GreenHorizon();
    GreenHorizon(const std::vector< Vector2<double> >& initial_points, Vector2<double> image_size);

    /**
      * Set the green horizon given a set of hull points. This method performs the necessary
      * interpolation to given a value for any horizontal position along the screen.
      * @param intial_points The scan points that form the hull.
      */
    void set(const std::vector< Vector2<double> >& initial_points, Vector2<double> image_size);

    //! Returns the y position of the horizon for a given x position.
    double getYFromX(int x) const;
    //! Returns whether the given point is below the horizon.
    bool isBelowHorizon(const Vector2<double>& pt) const;

    //! Returns the original hull points.
    const std::vector< Vector2<double> >& getOriginalPoints() const;
    //! Returns the interpolated points for the entire screen width.
    const std::vector<Vector2<double> >& getInterpolatedPoints() const;
    //! Returns a list of interpolated points with a given spacing.
    std::vector< Vector2<double> > getInterpolatedSubset(unsigned int spacing) const;

private:
    double interpolate(Vector2<double> p1, Vector2<double> p2, double x) const;

private:
    std::vector< Vector2<double> > original_points;      //! @variable The original hull points.
    std::vector< Vector2<double> > interpolated_points;  //! @variable The interpolated points.
};

#endif // GREENHORIZON_H
