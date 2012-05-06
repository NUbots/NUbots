#ifndef GOAL_H
#define GOAL_H

#include "Vision/VisionTypes/visionfieldobject.h"

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
    
    Goal();
    Goal(ID id);
    Goal(ID id, const Quad& corners);
    
    const vector<float>& getRelativeFieldCoords() const;
    
private:    
    ID m_id;
    Quad m_corners;
    int width;
};

#endif // GOAL_H
