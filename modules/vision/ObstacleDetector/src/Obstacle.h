#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <nuclear>
#include <armadillo>
#include <vector>

#include "messages/vision/ClassifiedImage.h"

#include "VisionFieldObject.h"

class Obstacle : public VisionFieldObject {
public:
    Obstacle(arma::vec2 position = arma::zeros<vec>(2), double width = 0, double height = 0, messages::vision::COLOUR_CLASS colour = messages::vision::UNKNOWN_COLOUR);

    /*!
      @brief pushes the obstacle to the external field objects.
      @param fieldobjects a pointer to the global list of field objects.
      @param timestamp the image timestamp.
      @return the success of the operation.
      */
    bool addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const;

    //! @brief applies a series of checks to decide if the obstacle is valid.
    bool check() const;
    
    virtual double findScreenError(VisionFieldObject* other) const;
    virtual double findGroundError(VisionFieldObject* other) const;

    //! @brief output stream operator.
    friend std::ostream& operator<< (std::ostream& output, const Obstacle& o);

    //! @brief output stream operator for a vector of obstacles.
    friend std::ostream& operator<< (std::ostream& output, const std::vector<Obstacle>& o);
    
private:
    /*!
      @brief calculates various positions values of the obstacle.
      @return whether the obstacle is valid.
      */
    bool calculatePositions();
    
public:
    messages::vision::COLOUR_CLASS m_colour;

private:
//    float d2p;                      //! @variable the distance of the obstacle in cm as found by the distance to point method
    double m_arcWidth;                //! @variable the angle subtended by the obstacle (based on the screen width)
};

#endif // OBSTACLE_H
