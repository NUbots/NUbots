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
    bool check() const;

    void setUnknown();
   
    friend ostream& operator<< (ostream& output, const Beacon& b);
    friend ostream& operator<< (ostream& output, const vector<Beacon>& b);
    
private:
    bool calculatePositions();
    float distanceToBeacon(float bearing, float elevation);
    
private:
    BeaconID m_id;
    Quad m_corners;
    Vector2<int> m_bottom_centre;
    
    float d2p;
    float width_dist;
};

#endif // BEACON_H
