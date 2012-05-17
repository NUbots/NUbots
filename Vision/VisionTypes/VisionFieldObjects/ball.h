#ifndef BALL_H
#define BALL_H

#include "Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h"

class Ball : public VisionFieldObject
{
public:
    Ball();
    Ball(int radius);
    
    Vector3<float> getRelativeFieldCoords() const;
    bool addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const;
    
private:
    void calculatePositions();
    
private:
    int m_radius;
};

#endif // BALL_H
