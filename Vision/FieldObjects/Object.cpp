//OBJECT.cpp
#include "Object.h"

Object::Object()
{
	sphericalPosition[0] = 0;
	sphericalPosition[1] = 0;
	sphericalPosition[2] = 0;
	sphericalError[0] = 0;
	sphericalError[1] = 0;
	sphericalError[2] = 0;
	isVisible = false;
	viewPosition[0] = 0;
	viewPosition[1] = 0;
	numberOfTimesSeen = 0;
	framesSinceLastSeen = 0;
	framesSeen = 0;
}

Object::~Object()
{	
}

void Object::UpdateVisualObject( Vector3<float> newSpherical,
                                Vector3<float> newSphericalError,
                                Vector2<int> newViewPosition)
{
	sphericalPosition = newSpherical;
	sphericalError = newSphericalError;
	viewPosition = newViewPosition;
	framesSinceLastSeen = 0;
	numberOfTimesSeen++;
	framesSeen++;
	isVisible = true;
}

void Object::ResetFrame()
{
	if(!isVisible)
	{
		framesSinceLastSeen++;
		framesSeen = 0;
	}
	isVisible = false;
}

void Object::CopyObject(Object sourceObject)
{
        sphericalPosition = sourceObject.getRelativeLocation();
        sphericalError = sourceObject.getRelativeLocationError();

        viewPosition = sourceObject.getViewPosition();
        isVisible = sourceObject.isObjectVisible();
}
/*
void Object::setRelativeLocationVariables(float distance, float bearing, float elevation)
{
	this.sphericalPosition[0] = distance;
	this.sphericalPosition[1] = bearing;
	this.sphericalPosition[2] = elevation;
}

void Object::setViewPositionVariables(int screenX, int screenY)
{
	this.viewPosition[0] = screenX;
	this.viewPosition[1] = screenY;
	
}*/
