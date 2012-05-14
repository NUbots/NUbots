#ifndef GOAL_H
#define GOAL_H

#include "VisionTypes/VisionFieldObjects/visionfieldobject.h"
#include "Infrastructure/FieldObjects/Object.h"

class Goal : public VisionFieldObject
{
public:
    enum ID {
        YellowLeft,
        YellowRight,
        YellowUnknown,
        BlueLeft,
        BlueRight,
        BlueUnknown,
        Invalid
    };
    
    static string getIDName(ID id);
       
    Goal(ID id=Invalid, const Quad& corners=Quad(0,0,0,0));
    
    void getRelativeFieldCoords(vector<float>& coords) const;
    bool addToExternalFieldObjects(FieldObjects *fieldobjects) const;
        
private:    
    ID m_id;
    Quad m_corners;
    int width;
};

#endif // GOAL_H
