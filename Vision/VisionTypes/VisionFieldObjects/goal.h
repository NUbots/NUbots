#ifndef GOAL_H
#define GOAL_H

#include "VisionTypes/VisionFieldObjects/visionfieldobject.h"
#include "VisionTypes/quad.h"
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
    
    enum DISTANCE_METHOD {
        Width,
        D2P
    };
    
    static const DISTANCE_METHOD METHOD = Width;
    
    static string getIDName(GoalID id);
       
    Goal(GoalID id=InvalidGoal, const Quad& corners=Quad(0,0,0,0));
    
    Vector3<float> getRelativeFieldCoords() const;
    bool addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const;
        
private:
    void calculatePositions();
    float distanceToGoal(float bearing, float elevation) const;
    
private:    
    GoalID m_id;
    Quad m_corners;
    Vector2<int> m_bottom_centre;
    Vector2<int> m_centre;
};

#endif // GOAL_H
