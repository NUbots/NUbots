#include "MobileObject.h"

MobileObject::MobileObject(int initID, const std::string& initName):
        Object(initID, initName)
{
    estimatedFieldLocationError[0] = 0;
    estimatedFieldLocationError[1] = 0;
    estimatedVelocity[0] = 0;
    estimatedVelocity[1] = 0;
    estimatedVelocityError[0] = 0;
    estimatedVelocityError[1] = 0;
    estimatedRelativeLocation[0] = 0;
    estimatedRelativeLocation[1] = 0;
    estimatedRelativeLocation[2] = 0;
}

MobileObject::MobileObject(const Vector2<float>& newEstimatedLocation, int initID, const std::string& initName):
                Object(initID, initName),
                estimatedFieldLocation(newEstimatedLocation)
{
        estimatedFieldLocationError[0] = 0;
        estimatedFieldLocationError[1] = 0;
        estimatedVelocity[0] = 0;
        estimatedVelocity[1] = 0;
        estimatedVelocityError[0] = 0;
        estimatedVelocityError[1] = 0;
        estimatedRelativeLocation[0] = 0;
        estimatedRelativeLocation[1] = 0;
        estimatedRelativeLocation[2] = 0;
	
}

MobileObject::MobileObject(const MobileObject& srcObj):
        Object(srcObj.getID(), srcObj.getName()),
        estimatedFieldLocation(srcObj.getEstimatedFieldLocation()),
        estimatedFieldLocationError(srcObj.getEstimatedFieldLocationError()),
        estimatedVelocity(srcObj.getEstimatedVelocity()),
        estimatedVelocityError(srcObj.getEstimatedVelocityError()),
        estimatedRelativeLocation(srcObj.getEstimatedRelativeLocation())
{
}

MobileObject::~MobileObject()
{
}

void MobileObject::updateAbsoluteLocation(const Vector2<float>& newAbsoluteLocation)
{
        estimatedFieldLocation = newAbsoluteLocation;
}
void MobileObject::updateAbsoluteLocationError(const Vector2<float>& newAbsoluteLocationError)
{
        estimatedFieldLocationError = newAbsoluteLocationError;
}
void MobileObject::updateObjectLocation(float x, float y, float sdx, float sdy)
{
        estimatedFieldLocation[0] = x;
        estimatedFieldLocation[1] = y;
        estimatedFieldLocationError[0] = sdx;
        estimatedFieldLocationError[1] = sdy;
}

void MobileObject::updateVelocity(const Vector2<float>& newVelocity)
{
        estimatedVelocity = newVelocity;
}
void MobileObject::updateVelocityError(const Vector2<float>& newVelocityError)
{
        estimatedVelocityError = newVelocityError;
	
}
void MobileObject::updateObjectVelocities(float velX, float velY, float sdVelX, float sdVelY)
{
        estimatedVelocity[0] = velX;
        estimatedVelocity[1] = velY;
        estimatedVelocityError[0] = sdVelX;
        estimatedVelocityError[1] = sdVelY;
}

void MobileObject::updateWorldModelRelativeLocation(const Vector3<float>& newWMRelLoc)
{
        estimatedRelativeLocation = newWMRelLoc;
}
void MobileObject::updateWorldModelRelativeLocationError(const Vector3<float>& newWMRelLocError)
{
        estimatedRelativeLocationError = newWMRelLocError;
}
void MobileObject::updateWorldModelRelativeVaribles(float wmDistance, float wmBearing, float wmElevation, float sdWmDistance, float sdWmBearing, float sdWmElevation )
{
        estimatedRelativeLocation[0] = wmDistance;
        estimatedRelativeLocation[1] = wmBearing;
        estimatedRelativeLocation[2] = wmElevation;
        estimatedRelativeLocationError[0] = sdWmDistance;
        estimatedRelativeLocationError[1] = sdWmBearing;
        estimatedRelativeLocationError[2] = sdWmElevation;
}
