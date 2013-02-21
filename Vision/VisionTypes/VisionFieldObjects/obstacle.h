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
    void printLabel(ostream& out) const {out << getVFOName(OBSTACLE) << " " << m_location.screen << " " << m_size_on_screen;}
    //! @brief Brief stream output for labelling purposes
    //void printLabelBrief(ostream& out) const {out << getVFOName(OBSTACLE) << " " << m_location_pixels;}
    Vector2<double> getShortLabel() const {return Vector2<double>(m_location.screen.x, m_location.screen.y);}

    double findError(const Vector2<double>& measured) const {return sqrt( pow(m_location.screen.x - measured.x,2) + pow(m_location.screen.y - measured.y,2));}

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
    
private:
    float d2p;                      //! @variable the distance of the obstacle in cm as found by the distance to point method
};

#endif // OBSTACLE_H
