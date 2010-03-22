#include "StationaryObject.h"

StationaryObject::StationaryObject(const Vector2<float>& initialFieldLocation, int id, const std::string& initName):
        Object(id, initName),
        fieldLocation(initialFieldLocation)
{
    estimatedRelativeLocation[0] = 0;
    estimatedRelativeLocation[1] = 0;
    estimatedRelativeLocation[2] = 0;
}

StationaryObject::StationaryObject(float x, float y, int id, const std::string& initName):
        Object(id, initName),
        fieldLocation(x,y)
{
    estimatedRelativeLocation[0] = 0;
    estimatedRelativeLocation[1] = 0;
    estimatedRelativeLocation[2] = 0;
}

StationaryObject::StationaryObject(const StationaryObject& otherObject):
        Object(otherObject.getID(), otherObject.getName()),
        fieldLocation(otherObject.getFieldLocation())
{

}

StationaryObject::~StationaryObject()
{

}

void StationaryObject::updateEstimatedRelativeLocation(const Vector3<float>& newWMRelLoc)
{
        estimatedRelativeLocation = newWMRelLoc;
}
void StationaryObject::updateEstimatedRelativeVariables(float distance, float bearing, float elevation)
{
        estimatedRelativeLocation[0] = distance;
        estimatedRelativeLocation[1] = bearing;
        estimatedRelativeLocation[2] = elevation;
}
