#include "StationaryObject.h"

StationaryObject::StationaryObject()
{
    ID = 0;
    AbsoluteLocation[0] = 0;
    AbsoluteLocation[1] = 0;
    WorldModelRelativeLocation[0] = 0;
    WorldModelRelativeLocation[1] = 0;
    WorldModelRelativeLocation[2] = 0;
}

StationaryObject::StationaryObject(int id, float x, float y): ID(id), AbsoluteLocation(x,y)
{
	WorldModelRelativeLocation[0] = 0;
	WorldModelRelativeLocation[1] = 0;
	WorldModelRelativeLocation[2] = 0;
}
StationaryObject::StationaryObject(int id, Vector2<float> newAbsoluteLocation):ID(id), AbsoluteLocation(newAbsoluteLocation)
{
	WorldModelRelativeLocation[0] = 0;
	WorldModelRelativeLocation[1] = 0;
	WorldModelRelativeLocation[2] = 0;
}

StationaryObject::StationaryObject(const StationaryObject& otherObject): ID(otherObject.getID()), AbsoluteLocation(otherObject.getAbsoluteLocation())
{

}

StationaryObject::~StationaryObject()
{

}

void StationaryObject::updateWorldModelRelativeLocation(Vector3<float> newWMRelLoc)
{
	WorldModelRelativeLocation = newWMRelLoc;
}
void StationaryObject::updateWorldModelRelativeVariables(float distance, float bearing, float elevation)
{
	WorldModelRelativeLocation[0] = distance;
	WorldModelRelativeLocation[1] = bearing;
	WorldModelRelativeLocation[2] = elevation;
}
