#include "beacon.h"
#include "debug.h"
#include "debugverbosityvision.h"

string Beacon::getIDName(ID id)
{
    switch(id) {
    case Yellow:    return "Yellow";
    case Blue:      return "Blue";
    case Unknown:   return "Unknown";
    case Invalid:   return "Invalid";
    }
}

Beacon::Beacon()
{
    Beacon(Invalid);
}

Beacon::Beacon(ID id)
{
    Beacon(id, Quad(0,0,0,0));
}

Beacon::Beacon(ID id, const Quad &corners)
{
    m_id = id;
    m_corners = corners;
    //SET WIDTH
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
    case Yellow:
        
        break;
    case Blue:
        
        break;
    case Unknown:
        
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

