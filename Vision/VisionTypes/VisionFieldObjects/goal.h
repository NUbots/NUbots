#ifndef GOAL_H
#define GOAL_H

#include "Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h"
#include "Vision/VisionTypes/quad.h"
#include "Infrastructure/FieldObjects/Object.h"
#include "Tools/Math/Vector2.h"

class Goal : public VisionFieldObject
{
public:
       
    Goal(VFO_ID id=INVALID, const Quad& corners=Quad());

    void setBase(Point base);

    //! @brief reutns the pixel locations of the corners.
    const Quad& getQuad() const;

    /*!
      @brief pushes the goal to the external field objects.
      @param fieldobjects a pointer to the global list of field objects.
      @param timestamp the image timestamp.
      @return the success of the operation.
      */
    bool addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const;
    //! @brief applies a series of checks to decide if the goal is valid.
    bool check() const;
        
    //! @brief Stream output for labelling purposes
    void printLabel(ostream& out) const {out << VFOName(m_id) << " " << m_location << " " << m_size_on_screen;}

    virtual double findScreenError(VisionFieldObject* other) const;
    virtual double findGroundError(VisionFieldObject* other) const;

    //! @brief output stream operator.
    friend ostream& operator<< (ostream& output, const Goal& g);
    //! @brief output stream operator for a vector of goals.
    friend ostream& operator<< (ostream& output, const vector<Goal>& g);    
    
private:
    /*!
      @brief calculates various positions values of the goal.
      @return whether the goal is valid.
      */
    bool calculatePositions();
    /*!
      @brief calculates distance to the goal based on the global goal distance metric.
      @param bearing the angle between the goal and the image centre in the xy plane.
      @param elevation the angle between the goal and the image centre in the xz plane.
      @return the distance to the goal in cm.
      */
//    double distanceToGoal(double bearing, double elevation);
    
private:
    Quad m_corners;                 //! @variable pixel locations of the corners

//public:
//    double width_dist,
//           d2p;
};

#endif // GOAL_H
