#ifndef WMPoint_H
#define WMPoint_H
#include "Tools/Math/Matrix.h"

/*!
  * A class to define a point used in a world model.
  */
class WMPoint
{
public:
    /*!
      @brief A constructor for WMPoint class.
      @param X The x ordinate of the point.
      @param Y The y ordinate of the point.
      @param Z The z ordinate of the point.
      */
    WMPoint(double X=0,double Y=0, double Z=0);

    /*!
      @brief A constructor for WMPoint class
      @param point A matrix of the coordinates of the new point.
      */
    WMPoint(Matrix point);

    /*!
      @brief Returns the x ordinate of the point.
      */
    double getx() {return x;}

    /*!
      @brief Returns the y ordinate of the point.
      */
    double gety() {return y;}

    /*!
      @brief Returns the z ordinate of the point.
      */
    double getz() {return z;}

    /*!
      @brief Returns the matrix representing the coordinates of the point.
      */
    Matrix get() {return coordinates;}

    /*!
      @brief Accepts a value and sets the x ordinate of the point to it.
      */
    void setx(double X);

    /*!
      @brief Accepts a value and sets the y ordinate of the point to it.
      */
    void sety(double Y);

    /*!
      @brief Accepts a value and sets the z ordinate of the point to it.
      */
    void setz(double Z);

private:
    double x;                       //!< x Ordinate of the point.
    double y;                       //!< y Ordinate of the point.
    double z;                       //!< z Ordinate of the point.
    Matrix coordinates;             //!< Matrix representing the coordinates of the point.
};
#endif // WMPoint_H
