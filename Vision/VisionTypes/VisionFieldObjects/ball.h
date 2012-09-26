#ifndef BALL_H
#define BALL_H

#include "Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h"

class Ball : public VisionFieldObject
{
public:
    
    Ball();
    Ball(PointType centre, int diameter);
    
    /*!
      @brief returns the radius.
      @return the radius of the ball in pixels.
      */
    float getRadius() const;
    
    //! @brief returns the field position relative to the robot.
    Vector3<float> getRelativeFieldCoords() const;
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
    void printLabel(ostream& out) {out << getVFOName(BALL) << " " << m_location_pixels << " " << m_diameter << endl;}

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
    float distanceToBall(float bearing, float elevation);
    
public:
    int m_diameter;     //! @variable the radius of the ball in pixels
    
private:
    float d2p;          //! @variable the distance of the ball in cm as found by the distance to point method
    float width_dist;   //! @variable the distance of the ball in cm as found by the width method.
};

#endif // BALL_H
