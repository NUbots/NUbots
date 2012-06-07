#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h"
#include "Vision/VisionTypes/quad.h"
#include "Tools/Math/Vector2.h"

class Obstacle : public VisionFieldObject
{
public:
    Obstacle(const PointType& position, int width, int height);
    
    Vector3<float> getRelativeFieldCoords() const;
    bool addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const;
    bool check() const;
    
    friend ostream& operator<< (ostream& output, const Obstacle& o);
    friend ostream& operator<< (ostream& output, const vector<Obstacle>& o);
    
private:
    void calculatePositions();
    
private:    
    Vector2<int> m_bottom_centre;
};

#endif // OBSTACLE_H
