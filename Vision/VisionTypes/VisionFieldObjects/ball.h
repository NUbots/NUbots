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
    
    friend ostream& operator<< (ostream& output, const Ball& b);
    friend ostream& operator<< (ostream& output, const vector<Ball>& b);
    
private:
    void calculatePositions();
    
private:
    int m_radius;
};

#endif // BALL_H
