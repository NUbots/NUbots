#ifndef BALL_H
#define BALL_H

#include "Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h"

class Ball : public VisionFieldObject
{
public:
    
    Ball();
    Ball(const PointType& centre, float radius);
    
    float getRadius() const;
    
    Vector3<float> getRelativeFieldCoords() const;
    bool addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const;
    void check();

    friend ostream& operator<< (ostream& output, const Ball& b);
    friend ostream& operator<< (ostream& output, const vector<Ball>& b);
    
private:
    void calculatePositions();
    float distanceToBall(float bearing, float elevation);
    
private:
    float m_radius;
    
    float d2p;
    float width_dist;
};

#endif // BALL_H
