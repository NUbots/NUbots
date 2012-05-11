#include "beacon.h"

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

void Beacon::getRelativeFieldCoords(vector<float> &coords) const
{
    
}
