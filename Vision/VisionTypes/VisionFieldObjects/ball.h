#ifndef BALL_H
#define BALL_H

#include "VisionTypes/VisionFieldObjects/visionfieldobject.h"

class Ball : public VisionFieldObject
{
public:
    Ball();
    Ball(int radius);
    
    void getRelativeFieldCoords(vector<float>& coords) const;
    bool addToExternalFieldObjects(FieldObjects *fieldobjects) const;
    
private:
    int m_radius;
};

#endif // BALL_H
