#ifndef GOAL_H
#define GOAL_H

#include "Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h"
#include "Vision/VisionTypes/quad.h"
#include "Infrastructure/FieldObjects/Object.h"
#include "Tools/Math/Vector2.h"

class Goal : public VisionFieldObject
{
public:
    enum GoalID {
        YellowLeftGoal,
        YellowRightGoal,
        YellowUnknownGoal,
        BlueLeftGoal,
        BlueRightGoal,
        BlueUnknownGoal,
        InvalidGoal
    };
    
    static string getIDName(GoalID id);
       
    Goal(GoalID id=InvalidGoal, const Quad& corners=Quad(0,0,0,0));

    //! @brief reutns the pixel locations of the corners.
    const Quad& getQuad() const;
    //! @brief returns the goal id - blue left, right or unknown and yellow left, right or unknown.
    GoalID getID() const;

    //! @brief returns the field position relative to the robot.
    Vector3<float> getRelativeFieldCoords() const;
    /*!
      @brief pushes the goal to the external field objects.
      @param fieldobjects a pointer to the global list of field objects.
      @param timestamp the image timestamp.
      @return the success of the operation.
      */
    bool addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const;
    //! @brief applies a series of checks to decide if the goal is valid.
    bool check() const;

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
    float distanceToGoal(float bearing, float elevation);
    
private:    
    GoalID m_id;                    //! @variable the goal id - blue left, right or unknown and yellow left, right or unknown.
    Quad m_corners;                 //! @variable pixel locations of the corners
    Vector2<int> m_bottom_centre;   //! @variable pixel location of the bottom centre

    float d2p;          //! @variable the distance of the goal in cm as found by the distance to point method
    float width_dist;   //! @variable the distance of the goal in cm as found by the width method.
};

#endif // GOAL_H
