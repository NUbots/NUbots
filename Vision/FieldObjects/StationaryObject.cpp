#include "StationaryObject.h"

StationaryObject::StationaryObject()
{
	AbsoluteLocation[0] = 0;
	AbsoluteLocation[1] = 0;
	WorldModelRelativeLocation[0] = 0;
	WorldModelRelativeLocation[1] = 0;
	WorldModelRelativeLocation[2] = 0;
}
StationaryObject::~StationaryObject()
{
	
}

StationaryObject::StationaryObject(float x, float y)
{
	AbsoluteLocation[0] = x;
	AbsoluteLocation[1] = y;
	WorldModelRelativeLocation[0] = 0;
	WorldModelRelativeLocation[1] = 0;
	WorldModelRelativeLocation[2] = 0;
}
StationaryObject::StationaryObject(Vector2<float> newAbsoluteLocation)
{
	AbsoluteLocation = newAbsoluteLocation;
	WorldModelRelativeLocation[0] = 0;
	WorldModelRelativeLocation[1] = 0;
	WorldModelRelativeLocation[2] = 0;
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
