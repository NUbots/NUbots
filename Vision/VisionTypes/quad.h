#ifndef QUAD_H
#define QUAD_H

#include <iostream>
#include <vector>
#include "Tools/Math/Vector2.h"
#include "Vision/basicvisiontypes.h"

class Quad
{
public:
    
    Quad();
    Quad(const Quad& other);
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
    
    Vector2<double> getTopCentre() const { return (tl + tr)*0.5; }   //! Returns the bottom centre pixel location of the Quad.
    Vector2<double> getBottomCentre() const { return (bl + br)*0.5; }   //! Returns the bottom centre pixel location of the Quad.

    Vector2<double> getCentre() const;         //! Returns the centre pixel location  of the Quad.

    Vector2<double> getBottomLeft() const {return bl;}     //! Returns the bottom left pixel location  of the Quad.
    Vector2<double> getBottomRight() const {return br;}     //! Returns the bottom right pixel location  of the Quad.
    Vector2<double> getTopLeft() const {return tl;}       //! Returns the top left pixel location  of the Quad.
    Vector2<double> getTopRight() const {return tr;}       //! Returns the top right pixel location  of the Quad.

    double getLeft() const {return 0.5*(bl.x + tl.x);}
    double getRight() const {return 0.5*(br.x + tr.x);}
    double getTop() const {return 0.5*(tl.y + tr.y);}
    double getBottom() const {return 0.5*(bl.y + br.y);}

    int getBaseWidth() const;                   //! Returns the base width of the Quad in pixels.
    int getTopWidth() const;                    //! Returns the top width of the Quad in pixels.

    int getLeftHeight() const;                  //! Returns the left height of the Quad in pixels.
    int getRightHeight() const;                 //! Returns the right height of the Quad in pixels.

    double getAverageWidth() const;                //! Returns the average width of the Quad in pixels.
    double getAverageHeight() const;               //! Returns the average height of the Quad in pixels.

    double area() const;

    bool overlapsHorizontally(const Quad& other) const;

    //! @brief output stream operator.
    friend std::ostream& operator<< (std::ostream& output, const Quad& g);
    //! @brief output stream operator for a vector of goals.
    friend std::ostream& operator<< (std::ostream& output, const std::vector<Quad>& g);

private:

    Vector2<double> bl,     //! @variable The bottom-left of the Quad.
                    br,    //! @variable The bottom-right of the Quad.
                    tr,       //! @variable The top-right of the Quad.
                    tl;        //! @variable The top-left of the Quad.
};

#endif // QUAD_H

