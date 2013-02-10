#ifndef GREENHORIZON_H
#define GREENHORIZON_H

#include <vector>

#include "Vision/basicvisiontypes.h"

using std::vector;

class GreenHorizon
{
public:
    GreenHorizon();
    GreenHorizon(const vector<Point>& initial_points, Point image_size);

    /**
      * Set the green horizon given a set of hull points. This method performs the necessary
      * interpolation to given a value for any horizontal position along the screen.
      * @param intial_points The scan points that form the hull.
      */
    void set(const vector<Point>& initial_points, Point image_size);

    //! Returns the y position of the horizon for a given x position.
    double getYFromX(int x) const;
    //! Returns whether the given point is below the horizon.
    bool isBelowHorizon(Point pt) const;

    //! Returns the original hull points.
    const vector<Point>& getOriginalPoints() const;
    //! Returns the interpolated points for the entire screen width.
    const vector<Point>& getInterpolatedPoints() const;
    //! Returns a list of interpolated points with a given spacing.
    vector<Point> getInterpolatedSubset(unsigned int spacing) const;

private:
    double interpolate(Point p1, Point p2, double x) const;

private:
    vector<Point> original_points;      //! @variable The original hull points.
    vector<Point> interpolated_points;  //! @variable The interpolated points.
};

#endif // GREENHORIZON_H
