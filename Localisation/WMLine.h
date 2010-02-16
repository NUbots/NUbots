#ifndef WMLine_H
#define WMLine_H
#include "WMPoint.h"

/*!
  * A class to define a line used in a world model
  */
class WMLine
{
public:
    /*!
      @brief A constructor for WMLine class.
      @param lineStart The point at the start of the line.
      @param lineStart The point at the end of the line.
      */
    WMLine(WMPoint lineStart = WMPoint(), WMPoint lineEnd = WMPoint());

    /*!
      @brief Return the point at the start of the line.
      */
    WMPoint getStart() {return start;}

    /*!
      @brief Return the point at the end of the line.
      */
    WMPoint getEnd() {return end;}

    /*!
      @brief Accept a point and set the start of the line to it
      @param point The new start point.
      */
    void setStart(WMPoint point);

    /*!
      @brief Accept a point and set the end of the line to it
      @param point The new end point.
      */
    void setEnd(WMPoint point);

    /*!
      @brief Reduces the line to fit inside a unit prism.
      */
    void normalise();

    /*!
      @brief Converts the line to screen coordinates.
      */
    void screenCoords();


private:
    WMPoint start;                      //!< Start of the Line.
    WMPoint end;                        //!< End of the Line.

};

/*!
  @brief Clips a line between two clipping planes in the z dimension.
  @param line The line to be clipped in 3D.
  @param near The closer clipping plane.
  @param far The furthur clipping plane.
  */
bool Clip3D(WMLine& line, double near, double far);

/*!
  @brief Clips a line to the viewing rectangle in the x and y dimensions.
  @param line The line to be clipped in 2D.
  */
bool Clip2D(WMLine& line);

#endif // WMLine_H
