#ifndef QUAD_H
#define QUAD_H

#include <opencv2/core/core.hpp>
#include "Tools/Math/Vector2.h"

#include "Vision/basicvisiontypes.h"

class Quad
{
public:
    
    Quad();
    Quad(const Quad& other);
    Quad(int left, int top, int right, int bottom);

    /**
      * Set the quad given the specified positions.
      * @param left     The left x pixel value.
      * @param top      The top y pixel value.
      * @param right    The right x pixel value.
      * @param bottom   The bottom y pixel value.
      */
    void set(int left, int top, int right, int bottom);
    

    Vector2<int> getBottomCentre() const;   //! Returns the bottom centre pixel location of the quad.

    Vector2<int> getCentre() const;         //! Returns the centre pixel location  of the quad.

    Vector2<int> getBottomLeft() const;     //! Returns the bottom left pixel location  of the quad.
    Vector2<int> getTopRight() const;       //! Returns the top right pixel location  of the quad.
    int getWidth() const;                   //! Returns the width of the quad in pixels.
    int getHeight() const;                  //! Returns the height of the quad in pixels.
    
    cv::Scalar getAsScalar() const;         //! Returns the quad as an OpenCV scalar
    
private:
    /**
      * Calculates the following after any of m_left, m_top, m_right or m_bottom have changed:
      *     - m_centre
      *     - m_bottom_centre
      *     - m_bottom_left
      *     - m_top_right
      */
    void recalculate();

    int m_left,     //! @variable The left of the quad.
        m_right,    //! @variable The right of the quad.
        m_top,      //! @variable The top of the quad.
        m_bottom;   //! @variable The bottom of the quad.

    cv::Scalar vals;       //! @variable A scalar holding the above values.

    Vector2<int> m_centre,          //! @variable The centre of the quad.
                 m_bottom_centre,   //! @variable The bottom centre of the quad.
                 m_bottom_left,     //! @variable The bottom left of the quad.
                 m_top_right;       //! @variable The top right of the quad.
};

#endif // QUAD_H
