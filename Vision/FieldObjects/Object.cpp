//OBJECT.cpp
#include "Object.h"

Object::Object(int initID, const std::string& initName):
        ID(initID),
        name(initName)
{
        measuredRelativePosition[0] = 0;
        measuredRelativePosition[1] = 0;
        measuredRelativePosition[2] = 0;
        relativeMeasurementError[0] = 0;
        relativeMeasurementError[1] = 0;
        relativeMeasurementError[2] = 0;
	isVisible = false;
        imagePosition[0] = 0;
        imagePosition[1] = 0;
	numberOfTimesSeen = 0;
	framesSinceLastSeen = 0;
	framesSeen = 0;
}

Object::~Object()
{	
}

void Object::UpdateVisualObject(    const Vector3<float>& newMeasured,
                                    const Vector3<float>& newMeasuredError,
                                    const Vector2<int>& newImagePosition)
{
        measuredRelativePosition = newMeasured;
        relativeMeasurementError = newMeasuredError;
        imagePosition = newImagePosition;
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

void Object::CopyObject(const Object& sourceObject)
{
        measuredRelativePosition = sourceObject.getMeasuredRelativeLocation();
        relativeMeasurementError = sourceObject.getRelativeMeasurementError();

        imagePosition = sourceObject.getImagePosition();
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
