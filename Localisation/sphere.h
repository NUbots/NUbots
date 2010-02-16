#ifndef SPHERE_H
#define SPHERE_H

#include "WMLine.h"
/*!
  * A class to define a sphere used in a world model.
  */
class Sphere
{
public:
    /*!
      @brief A constructor for Sphere class.
      @param x The x ordinate of the centre of the sphere.
      @param y The y ordinate of the centre of the sphere.
      */
    Sphere(double x = 0, double y = 0);

    /*!
      @brief Returns the centre of the sphere as a world model point.
      */
    WMPoint getCentre() {return centre;}

    /*!
      @brief Accepts a world model point and sets the centre of the sphere to it.
      @param newCentre The new centre of the sphere.
      */
    void setCentre(WMPoint newCentre);

    /*!
      @brief Returns a horizontal line representing the top of a sphere.
      */
    WMLine getTop() {return top;}

    /*!
      @brief Returns a horizontal line representing the bottom of a sphere.
      */
    WMLine getBottom() {return bottom;}

    /*!
      @brief Returns the centre of a sphere as a line with the same start and end points.
      */
    WMLine getMid() {return mid;}

    /*!
      @brief Creates the top, bottom and middle lines.
      */
    void createSides();

private:
    WMPoint centre;                     //!< Centre of the sphere.
    WMLine top;                         //!< Line representing the top of the sphere.
    WMLine bottom;                      //!< Line representing the bottom of the sphere.
    WMLine mid;                         //!< Line representing the centre of the sphere.
};

/*!
  @brief Determines if a sphere lies between two values in the z dimension.
  @param sphere The sphere to trivially clip in 3D.
  @param near The closer clipping plane.
  @param far The furthur clipping plane.
  */
bool Clip3D(Sphere sphere,double near, double far);
#endif // SPHERE_H
