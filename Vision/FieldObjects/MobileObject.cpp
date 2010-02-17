#include "MobileObject.h"
MobileObject::MobileObject()
{
	AbsoluteLocation[0] = 0;
	AbsoluteLocation[1] = 0;
	AbsoluteLocationError[0] = 0;
	AbsoluteLocationError[1] = 0;
	Velocity[0] = 0;
	Velocity[1] = 0;
	VelocityError[0] = 0;
	VelocityError[1] = 0;
	WorldModelRelativeLocation[0] = 0;
	WorldModelRelativeLocation[1] = 0;
	WorldModelRelativeLocation[2] = 0;
}
MobileObject::~MobileObject()
{
}
MobileObject::MobileObject(Vector2<float> newAbsoluteLocation)
{
	AbsoluteLocation = newAbsoluteLocation;
	AbsoluteLocationError[0] = 0;
	AbsoluteLocationError[1] = 0;
	Velocity[0] = 0;
	Velocity[1] = 0;
	VelocityError[0] = 0;
	VelocityError[1] = 0;
	WorldModelRelativeLocation[0] = 0;
	WorldModelRelativeLocation[1] = 0;
	WorldModelRelativeLocation[2] = 0;
	
}
MobileObject::MobileObject(float x, float y)
{
	AbsoluteLocation[0] = x;
	AbsoluteLocation[1] = y;
	AbsoluteLocationError[0] = 0;
	AbsoluteLocationError[1] = 0;
	Velocity[0] = 0;
	Velocity[1] = 0;
	VelocityError[0] = 0;
	VelocityError[1] = 0;
	WorldModelRelativeLocation[0] = 0;
	WorldModelRelativeLocation[1] = 0;
	WorldModelRelativeLocation[2] = 0;
}
void MobileObject::updateAbsoluteLocation(Vector2<float> newAbsoluteLocation)
{
	AbsoluteLocation = newAbsoluteLocation;
}
void MobileObject::updateAbsoluteLocationError(Vector2<float> newAbsoluteLocationError)
{
	AbsoluteLocationError = newAbsoluteLocationError;
}
void MobileObject::updateObjectLocation(float x, float y, float sdx, float sdy)
{
	AbsoluteLocation[0] = x;
	AbsoluteLocation[1] = y;
	AbsoluteLocationError[0] = sdx;
	AbsoluteLocationError[1] = sdy;
}

void MobileObject::updateVelocity(Vector2<float> newVelocity)
{
	Velocity = newVelocity;
}
void MobileObject::updateVelocityError(Vector2<float> newVelocityError)
{
	VelocityError = newVelocityError;
	
}
void MobileObject::updateObjectVelocities(float velX, float velY, float sdVelX, float sdVelY)
{
	Velocity[0] = velX;
	Velocity[1] = velY;
	VelocityError[0] = sdVelX;
	VelocityError[1] = sdVelY;
}

void MobileObject::updateWorldModelRelativeLocation(Vector3<float> newWMRelLoc)
{
	WorldModelRelativeLocation = newWMRelLoc;
}
void MobileObject::updateWorldModelRelativeLocationError(Vector3<float> newWMRelLocError)
{
	WorldModelRelativeLocationError = newWMRelLocError;
}
void MobileObject::updateWorldModelRelativeVaribles(float wmDistance, float wmBearing, float wmElevation, float sdWmDistance, float sdWmBearing, float sdWmElevation )
{
	WorldModelRelativeLocation[0] = wmDistance;
	WorldModelRelativeLocation[1] = wmBearing;
	WorldModelRelativeLocation[2] = wmElevation;
	WorldModelRelativeLocationError[0] = sdWmDistance;
	WorldModelRelativeLocationError[1] = sdWmBearing;
	WorldModelRelativeLocationError[2] = sdWmElevation;
}
