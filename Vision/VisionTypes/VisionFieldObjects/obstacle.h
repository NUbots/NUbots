#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h"
#include "Vision/VisionTypes/quad.h"
#include "Tools/Math/Vector2.h"

class Obstacle : public VisionFieldObject
{
public:
    Obstacle(const PointType& position, int width, int height);

    //! @brief returns the field position relative to the robot.
    Vector3<float> getRelativeFieldCoords() const;
    /*!
      @brief pushes the obstacle to the external field objects.
      @param fieldobjects a pointer to the global list of field objects.
      @param timestamp the image timestamp.
      @return the success of the operation.
      */
    bool addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const;
    //! @brief applies a series of checks to decide if the obstacle is valid.
    bool check() const;

    //! @brief output stream operator.
    friend ostream& operator<< (ostream& output, const Obstacle& o);
    //! @brief output stream operator for a vector of obstacles.
    friend ostream& operator<< (ostream& output, const vector<Obstacle>& o);
    
private:
    /*!
      @brief calculates various positions values of the obstacle.
      @return whether the obstacle is valid.
      */
    bool calculatePositions();
    /*!
      @brief calculates distance to the obstacle based on the global obstacle distance metric.
      @param bearing the angle between the obstacle and the image centre in the xy plane.
      @param elevation the angle between the obstacle and the image centre in the xz plane.
      @return the distance to the obstacle in cm.
      */
    float distanceToObstacle(float bearing, float elevation);
    
private:
    Vector2<int> m_bottom_centre;   //! @variable pixel location of the bottom centre
    float d2p;                      //! @variable the distance of the obstacle in cm as found by the distance to point method
};

#endif // OBSTACLE_H
