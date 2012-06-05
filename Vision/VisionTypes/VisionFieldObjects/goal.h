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
    
    enum GOAL_DISTANCE_METHOD {
        Width,
        D2P,
        Average,
        Least
    };
    
    static const GOAL_DISTANCE_METHOD METHOD = Least;
    
    static string getIDName(GoalID id);
       
    Goal(GoalID id=InvalidGoal, const Quad& corners=Quad(0,0,0,0));
    
    const Quad& getQuad() const;
    GoalID getID() const;
    
    Vector3<float> getRelativeFieldCoords() const;
    bool addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const;
    bool check() const;
    
    friend ostream& operator<< (ostream& output, const Goal& g);
    friend ostream& operator<< (ostream& output, const vector<Goal>& g);    
    
private:
    void calculatePositions();
    float distanceToGoal(float bearing, float elevation);
    
private:    
    GoalID m_id;
    Quad m_corners;
    Vector2<int> m_bottom_centre;
    
    float d2p;
    float width_dist;
};

#endif // GOAL_H
