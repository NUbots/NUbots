#ifndef BEACON_H
#define BEACON_H

#include "VisionTypes/VisionFieldObjects/visionfieldobject.h"
#include "Vision/VisionTypes/quad.h"

class Beacon : public VisionFieldObject
{
public:
    enum ID {
        Yellow,
        Blue,
        Unknown,
        Invalid
    };
    
    static string getIDName(ID id);
    
    Beacon();
    Beacon(ID id);
    Beacon(ID id, const Quad& corners);
    
    Vector3<float> getRelativeFieldCoords() const;
    bool addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const;
    
private:
    void calculatePositions();
    
private:
    ID m_id;
    Quad m_corners;
    int width;
};

#endif // BEACON_H
