#ifndef BALL_H
#define BALL_H

#include "Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h"

class Ball : public VisionFieldObject
{
public:
    
    Ball();
    Ball(Point centre, double diameter);
    
    /*!
      @brief returns the radius.
      @return the radius of the ball in pixels.
      */
    float getRadius() const;

    /*!
      @brief pushes the ball to the external field objects.
      @param fieldobjects a pointer to the global list of field objects.
      @param timestamp the image timestamp.
      @return the success of the operation.
      */
    bool addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const;

    //! @brief applies a series of checks to decide if the ball is valid.
    bool check() const;
    
    //! @brief Stream output for labelling purposes
    void printLabel(ostream& out) const {out << getVFOName(BALL) << " " << m_location_pixels << " " << m_diameter;}
    //! @brief Brief stream output for labelling purposes
    //void printLabelBrief(ostream& out) const {out << getVFOName(BALL) << " " << m_location_pixels;}
    Vector2<double> getShortLabel() const {return Vector2<double>(m_location_pixels.x, m_location_pixels.y);}

    double findError(const Vector2<double>& measured) const {return sqrt( pow(m_location_pixels.x - measured.x,2) + pow(m_location_pixels.y - measured.y,2));}

    void render(cv::Mat& mat) const;
    
    //! @brief output stream operator
    friend ostream& operator<< (ostream& output, const Ball& b);
    //! @brief output stream operator for a vector of balls
    friend ostream& operator<< (ostream& output, const vector<Ball>& b);
    
private:
    /*!
      @brief calculates various positions values of the ball.
      @return whether the ball is valid.
      */
    bool calculatePositions();
    /*!
      @brief calculates distance to the ball based on the global ball distance metric.
      @param bearing the angle between the ball and the image centre in the xy plane.
      @param elevation the angle between the ball and the image centre in the xz plane.
      @return the distance to the ball in cm.
      */
    double distanceToBall(double bearing, double elevation);
    
public:
    int m_diameter;     //! @variable the radius of the ball in pixels
    
private:
    float d2p;          //! @variable the distance of the ball in cm as found by the distance to point method
    float width_dist;   //! @variable the distance of the ball in cm as found by the width method.
};

#endif // BALL_H
