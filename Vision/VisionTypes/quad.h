#ifndef QUAD_H
#define QUAD_H

#include "Tools/Math/Vector2.h"

#include "Vision/basicvisiontypes.h"

class Quad
{
public:
    
    Quad();
    Quad(const Quad& other);
    Quad(int left, int top, int right, int bottom);
    Quad(Vector2<double> bottom_left, Vector2<double> top_left, Vector2<double> top_right, Vector2<double> bottom_right);

    /**
      * Sets the Quad as a screen aligned rectangle given the specified positions.
      * @param left     The left x pixel value.
      * @param top      The top y pixel value.
      * @param right    The right x pixel value.
      * @param bottom   The bottom y pixel value.
      */
    void set(int left, int top, int right, int bottom);

    /**
      * Sets the Quad given the specified corners.
      * @param bottom_left  The bottom left corner.
      * @param top_left     The top left corner.
      * @param top_right    The top right corner.
      * @param bottom_right The bottom right corner.
      */
    void set(Vector2<double> bottom_left, Vector2<double> top_left, Vector2<double> top_right, Vector2<double> bottom_right);
    

    Vector2<double> getBottomCentre() const;   //! Returns the bottom centre pixel location of the Quad.

    Vector2<double> getCentre() const;         //! Returns the centre pixel location  of the Quad.

    Vector2<double> getBottomLeft() const {return m_bottom_left;}     //! Returns the bottom left pixel location  of the Quad.
    Vector2<double> getBottomRight() const {return m_bottom_right;}     //! Returns the bottom right pixel location  of the Quad.
    Vector2<double> getTopLeft() const {return m_top_left;}       //! Returns the top left pixel location  of the Quad.
    Vector2<double> getTopRight() const {return m_top_right;}       //! Returns the top right pixel location  of the Quad.

    double getLeft() const {return 0.5*(m_bottom_left.x + m_top_left.x);}
    double getRight() const {return 0.5*(m_bottom_right.x + m_top_right.x);}
    double getTop() const {return 0.5*(m_top_left.y + m_top_right.y);}
    double getBottom() const {return 0.5*(m_bottom_left.y + m_bottom_right.y);}

    int getBaseWidth() const;                   //! Returns the base width of the Quad in pixels.
    int getTopWidth() const;                    //! Returns the top width of the Quad in pixels.

    int getLeftHeight() const;                  //! Returns the left height of the Quad in pixels.
    int getRightHeight() const;                 //! Returns the right height of the Quad in pixels.

    double getAverageWidth() const;                //! Returns the average width of the Quad in pixels.
    double getAverageHeight() const;               //! Returns the average height of the Quad in pixels.

private:

    Vector2<double> m_bottom_left,     //! @variable The left of the Quad.
                    m_bottom_right,    //! @variable The right of the Quad.
                    m_top_right,       //! @variable The top of the Quad.
                    m_top_left;        //! @variable The bottom of the Quad.
};

#endif // QUAD_H

