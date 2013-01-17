/*!
  @file Line.h
  @author Steven Nicklin
  @brief Definition of the Point and Line class.
*/

#ifndef LINE_H_DEFINED
#define LINE_H_DEFINED

#include <iostream>
#include "Tools/Math/Vector2.h"

/*!
  @brief Class representing a 2 dimensional line in the form Ax + By = C.
  */

typedef Vector2<double> Point;

class Line
{
  public:
    //! Default Constructor.
    Line();
    //! Constructor with intialising points. A line is created through these 2 points.
    Line(Point p1, Point p2);
    //! Constructor with intialising values.
    Line(double rho, double phi);
    //! Destructor
    ~Line();
// Make line. Form: Ax + By = C
    /*!
      @brief Assign the line by giving the equation in the form Ax + By = C.
      @param A The x coefficient.
      @param B The y coefficient.
      @param C The constant result.
      @return True if a valid line was assigned using the equation. False otherwise.
      */
    bool setLine(double A, double B, double C);

    /*!
      @brief Assign the line by giving the equation in the Hesse standard form.
      @param rho The length of the normal.
      @param phi The angle the normal makes with the x_axis.
      @return True if a valid line was assigned using the equation. False otherwise.
      */
    bool setLine(double rho, double phi);
    /*!
      @brief Assign the line by giving two points through wich to form the line.
      @param p1 The first point.
      @param p2 The second point.
      @return True if a valid line was assigned using the points. False otherwise.
      */
    bool setLineFromPoints(Point p1, Point p2);
    /*!
      @brief Normalises the rho/phi values.
      */
    void normaliseRhoPhi();
    /*!
      @brief Copy the contents of another line to assign the equation for this line.
      @param source The source from which to copy the equation data.
      @return True if a valid line was assigned from the existing line. False otherwise.
      */
    bool copy(const Line& source);
// Get line equation values.
    /*!
      @brief retreive the A value of the line equation.
      @return The A value of the line equation.
      */
    double getA() const;
    /*!
      @brief retreive the B value of the line equation.
      @return The B value of the line equation.
      */
    double getB() const;
    /*!
      @brief retreive the C value of the line equation.
      @return The C value of the line equation.
      */
    double getC() const;

// Check properties.
    /*!
      @brief Find if the line is horizontal.

      A horizontal line has a constant y value.
      @return True if the line is horizontal. False otherwise.
      */
    bool isHorizontal() const;
    /*!
      @brief Find if the line is vertical.

      A vertical line has a constant x value.
      @return True if the line is vertical. False otherwise.
      */
    bool isVertical() const;
    /*!
      @brief Find if the line contains a valid line equation.
      @return True if the line is valid. False otherwise.
      */
    bool isValid() const;
// Calculate attributes.
    /*!
      @brief Find the x value at a given y value of the current line.
      @param y The y position.
      @return The x value of the line at y.
      */
    double findXFromY(double y) const;
    /*!
      @brief Find the y value at a given x value of the current line.
      @param x The x position.
      @return The y value of the line at x.
      */
    double findYFromX(double x) const;
    /*!
      @brief Find the gradient of the line.

      The gradient is the m variable if the line is put into the form y = mx + b.
      @return The gradient of the line.
      */
    double getGradient() const;
    /*!
      @brief Find the angle of the line in radians.

      The angle given is the angle from the x axis to the line in an anticlockwise direction.
      @return The angle of the line in radians.
      */
    double getAngle() const;
    /*!
      @brief Find the x intercept of the line.

      The x intercept is the y value of the line at x = 0.
      @return The x intercept.
      */
    double getXIntercept() const;
    /*!
      @brief Find the y intercept of the line.

      The y intercept is the x value of the line at y = 0.
      @return The y intercept.
      */
    double getYIntercept() const;
    /*!
      @brief Find the distance between the the line and the point.
      @param point The point to find the distance to.
      @return The distance from the line to the point.
      */
    double getLinePointDistance(Point point) const;
    /*Added by Shannon*/
    /*!
      @brief retreive the normaliser for the coefficients of the line equation - sqrt(A^2 + B^2).
      @return sqrt(A^2 + B^2).
      */
    double getNormaliser() const;
    /*!
      @brief Find the signed distance between the the line and the point.
      @param point The point to find the distance to.
      @return The signed distance from the line to the point.
      */
    double getSignedLinePointDistance(Point point) const;
    /*!
      @brief Find the smallest angle between this and the given line.
      @param other The other line.
      @return The acute angle between the lines.
      */
    double getAngleBetween(Line other) const;
    /*!
      @brief Retrieve the Rho value of the line equation (normal form)
      @return Rho.
      */
    double getRho() const;
    /*!
      @brief Retrieve the Phi value of the line equation (normal form)
      @return Phi.
      */
    double getPhi() const;
    /*!
      @brief Projects the point onto the line.
      @param pt The point to project.
      @return The projected point.
      */
    Point projectOnto(Point pt) const;
    /*!
      @brief Finds the intersection of the two lines.
      @param other The other line.
      @param pt The resulting point.
      @return Whether the lines intersect.
      */
    bool getIntersection(const Line& other, Point& pt) const;
    /*Added by Shannon*/

// Overloaded functions
    /*!
      @brief Equality operator
      @return True of the two lines are equal. False if they are not.
      */
    friend bool operator ==(const Line& line1, const Line& line2);
    /*!
      @brief Unequality operator
      @return False of the two lines are equal. True if they are not.
      */
    friend bool operator !=(const Line& line1, const Line& line2);
    /*!
      @brief Greater than operator
      @return True if line1 grad > line2 grad, if == then true if
            line1 y-int > line2 y-int. False otherwise.
      */
    friend bool operator > (const Line& line1, const Line& line2);
    /*!
      @brief Output stream operator
      */
    friend std::ostream& operator<< (std::ostream& output, const Line& l);
    
  protected:
    double m_A; //! The lines A value.
    double m_B; //! The lines B value.
    double m_C; //! The lines C value.
    double m_rho;
    double m_phi;
    double m_normaliser;
    /*!
      @brief Determine if the line represented by the given equation is valid.
      @param A The A value of the line equation.
      @param B The B value of the line equation.
      @param C The C value of the line equation.
      @return True if the equation is valid. False if it is not.
      */
    bool isValid(double A, double B, double C) const;
};

#endif
