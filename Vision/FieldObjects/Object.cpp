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
    timeLastSeen = 0;  
    timeSinceLastSeen = 0;
    timeSeen = 0;         
    previousFrameTimestamp = 0;
}

Object::~Object()
{	
}

/*! @brief Preprocess the field object prior to updating visual information
 */
void Object::preProcess(const float timestamp)
{
}

/*! @brief Updates the field object's measured position, measured error, position in image, and last seen time
    @param newMeasured the measured visual relative position (distance, bearing, elevation)
    @param newMeasuredError the error in the measured visual relative position (distance, bearing, elevation)
    @param newImagePosition the position in the image the object was seen (x pixels, y pixels)
    @param newSizeOnScreen is thesize of the object on the screen (pixels wide, pixels high)
    @param timestamp the current time that is used to set the last seen time
 */
void Object::UpdateVisualObject(const Vector3<float>& newMeasured,
                                const Vector3<float>& newMeasuredError,
                                const Vector2<int>& newImagePosition,
                                const Vector2<int>& newSizeOnScreen,
                                const float timestamp)
{
    measuredRelativePosition = newMeasured;
    relativeMeasurementError = newMeasuredError;
    imagePosition = newImagePosition;
    sizeOnScreen = newSizeOnScreen;

    isVisible = true;
    timeLastSeen = timestamp;
    timeSinceLastSeen = 0;
    timeSeen += timestamp - previousFrameTimestamp;
}

/*! @brief Postprocess the field object after updating visual information
    @param timestamp the current image timestamp
 
    This function sets isVisible, timeSinceLastSeen and timeSeen if the object has received no new visual information.
    We also always set the previousFrameTimestamp here.
 */
void Object::postProcess(const float timestamp)
{
    if (timeLastSeen != timestamp)
    {
        isVisible = false;
        timeSinceLastSeen = timestamp - timeLastSeen;
        timeSeen = 0;
    }
    previousFrameTimestamp = timestamp;
}

void Object::CopyObject(const Object& sourceObject)
{
        measuredRelativePosition = sourceObject.getMeasuredRelativeLocation();
        relativeMeasurementError = sourceObject.getRelativeMeasurementError();

        imagePosition = sourceObject.getImagePosition();
        sizeOnScreen.x = sourceObject.getObjectWidth();
        sizeOnScreen.y = sourceObject.getObjectHeight();
        isVisible = sourceObject.isObjectVisible();
        timeLastSeen = sourceObject.TimeSeen();
        timeSeen += sourceObject.TimeSeen() - previousFrameTimestamp;
        timeSinceLastSeen = 0;
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
