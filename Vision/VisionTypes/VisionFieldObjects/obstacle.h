#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h"
#include "Vision/VisionTypes/quad.h"
#include "Tools/Math/Vector2.h"

class Obstacle : public VisionFieldObject
{
public:
    Obstacle(Point position=Point(0,0), double width=0, double height=0);

    /*!
      @brief pushes the obstacle to the external field objects.
      @param fieldobjects a pointer to the global list of field objects.
      @param timestamp the image timestamp.
      @return the success of the operation.
      */
    bool addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const;
    //! @brief applies a series of checks to decide if the obstacle is valid.
    bool check() const;
    
    //! @brief Stream output for labelling purposes
    void printLabel(std::ostream& out) const {out << VFOName(OBSTACLE) << " " << m_location << " " << m_size_on_screen;}

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
    
private:
//    float d2p;                      //! @variable the distance of the obstacle in cm as found by the distance to point method
    double m_arc_width;               //! @variable the angle subtended by the obstacle (based on the screen width)
};

#endif // OBSTACLE_H
