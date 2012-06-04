#ifndef BALL_H
#define BALL_H

#include "Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h"

class Ball : public VisionFieldObject
{
public:
    
    enum BALL_DISTANCE_METHOD {
        Width,
        D2P,
        Average
    };
    
    static const BALL_DISTANCE_METHOD METHOD = Width;
    
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
    float distanceToBall(float bearing, float elevation) const;
    
private:
    float m_radius;
};

#endif // BALL_H
