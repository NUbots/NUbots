#ifndef BEACON_H
#define BEACON_H

#include "Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h"
#include "Vision/VisionTypes/quad.h"

class Beacon : public VisionFieldObject
{
public:
    enum BeaconID {
        YellowBeacon,
        BlueBeacon,
        UnknownBeacon,
        InvalidBeacon
    };
    
    static string getIDName(BeaconID id);
    
    Beacon(BeaconID id = InvalidBeacon, const Quad& corners = Quad(0,0,0,0));
    
    const Quad& getQuad() const;
    BeaconID getID() const;
    
    Vector3<float> getRelativeFieldCoords() const;
    bool addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const;
   
    friend ostream& operator<< (ostream& output, const Beacon& b);
    friend ostream& operator<< (ostream& output, const vector<Beacon>& b);
    
private:
    void calculatePositions();
    
private:
    BeaconID m_id;
    Quad m_corners;
    int width;
};

#endif // BEACON_H
