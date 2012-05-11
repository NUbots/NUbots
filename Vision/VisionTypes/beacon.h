#ifndef BEACON_H
#define BEACON_H

#include "Vision/VisionTypes/visionfieldobject.h"

class Beacon : public VisionFieldObject
{
public:
    enum ID {
        Yellow,
        Blue,
        Unknown,
        Invalid
    };
    
    Beacon();
    Beacon(ID id);
    Beacon(ID id, const Quad& corners);
    
    void getRelativeFieldCoords(vector<float>& coords) const;
    
private:
    ID m_id;
    Quad m_corners;
    int width;
};

#endif // BEACON_H
