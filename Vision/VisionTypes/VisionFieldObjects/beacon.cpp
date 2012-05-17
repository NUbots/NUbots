#include "beacon.h"
#include "debug.h"
#include "debugverbosityvision.h"

string Beacon::getIDName(BeaconID id)
{
    switch(id) {
    case YellowBeacon:    return "YellowBeacon";
    case BlueBeacon:      return "BlueBeacon";
    case UnknownBeacon:   return "UnknownBeacon";
    case InvalidBeacon:   return "InvalidBeacon";
    }
}

Beacon::Beacon(BeaconID id, const Quad &corners)
{
    m_id = id;
    m_corners = corners;
    //SET WIDTH
}

const Quad& Beacon::getQuad() const
{
    return m_corners;
}

Beacon::BeaconID Beacon::getID() const
{
    return m_id;
}

Vector3<float> Beacon::getRelativeFieldCoords() const
{
    return m_spherical_position;
}

bool Beacon::addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const
{
    #if VISION_FIELDOBJECT_VERBOSITY > 1
        debug << "Beacon::addToExternalFieldObjects - m_id: " << getIDName(m_id) << endl;
    #endif
        
    switch(m_id) {
    case YellowBeacon:
        
        break;
    case BlueBeacon:
        
        break;
    case UnknownBeacon:
        
        break;
    default:
        //invalid object - do not push to fieldobjects
        errorlog << "Beacon::addToExternalFieldObjects - attempt to add invalid beacon object" << endl;
        #if VISION_FIELDOBJECT_VERBOSITY > 1
            debug << "Beacon::addToExternalFieldObjects - attempt to add invalid beacon object" << endl;
        #endif
    }
}

void Beacon::calculatePositions()
{
    //! @todo implement 
    m_spherical_position[0] = 0; //dist
    m_spherical_position[1] = 0; //bearing
    m_spherical_position[2] = 0; //elevation
    
    m_location_angular.x = 0; //bearing
    m_location_angular.y = 0; //elevation
}

