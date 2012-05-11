#ifndef BALL_H
#define BALL_H

#include "Vision/VisionTypes/visionfieldobject.h"

class Ball : public VisionFieldObject
{
public:
    Ball();
    Ball(int radius);
    
    void getRelativeFieldCoords(vector<float>& coords) const;
    
private:
    int m_radius;
};

#endif // BALL_H
