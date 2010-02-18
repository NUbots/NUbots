#ifndef CYLINDER_H
#define CYLINDER_H
#include "WMLine.h"
/*!
  * A class to define a cylinder in a world model.
  */
class Cylinder
{
public:
    /*!
      @brief A constructor for Cylinder class.
      @param start The centre of one end of the cylinder.
      @param end The centre of the other end of the cylinder.
      */
    Cylinder(WMPoint start = WMPoint(), WMPoint end = WMPoint());

    /*!
      @brief Returns the line through the centre of the cylinder.
      */
    WMLine getMid() {return mid;}

    /*!
      @brief Returns the line representing the left edge of the cylinder.
      */
    WMLine getLeft() {return left;}

    /*!
      @brief Returns the line representing the right edge of the cylinder.
      */
    WMLine getRight() {return right;}

    /*!
      @brief Accepts two points and sets the middle of the cylinder as the line between them.
      @param start The centre of one new end of the cylinder.
      @param end The centre of the other new end of the cylinder.
      */
    void setMid(WMPoint start, WMPoint end);

    /*!
      @brief Accepts two points and sets the left edge of the cylinder as the line between them.
      @param start The new start of the left edge of the cylinder.
      @param end The new end of the left edge of the cylinder.
      */
    void setLeft(WMPoint start, WMPoint end);

    /*!
      @brief Accepts two points and sets the right edge of the cylinder as the line between them.
      @param start The new start of the right edge of the cylinder.
      @param end The new end of the right edge of the cylinder.
      */
    void setRight(WMPoint start, WMPoint end);

    /*!
      @brief Calculate the left and right edge of the cylinder visible to the robot.
      */
    void createSides();
private:
    WMLine left;                    //!< Line representing the left edge of the cylinder visible in 2D
    WMLine right;                   //!< Line representing the right edge of the cylinder visible in 2D
    WMLine mid;                     //!< Line representing the centre axis of the cylinder.
};

#endif // CYLINDER_H
