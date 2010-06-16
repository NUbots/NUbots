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

    estimatedRelativeLocation[0] = 0;
    estimatedRelativeLocation[1] = 0;
    estimatedRelativeLocation[2] = 0;

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
                                const Vector2<float>& newImagePositionAngle,
                                const Vector2<int>& newImagePosition,
                                const Vector2<int>& newSizeOnScreen,
                                const float timestamp)
{
    measuredRelativePosition = newMeasured;
    relativeMeasurementError = newMeasuredError;
    imagePositionAngle = newImagePositionAngle;
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

//! Functions for Localisationt to update the object
void Object::updateEstimatedRelativeLocation(const Vector3<float>& newWMRelLoc)
{
        estimatedRelativeLocation = newWMRelLoc;
}
void Object::updateEstimatedRelativeVariables(float distance, float bearing, float elevation)
{
        estimatedRelativeLocation[0] = distance;
        estimatedRelativeLocation[1] = bearing;
        estimatedRelativeLocation[2] = elevation;
}


std::ostream& operator<< (std::ostream& output, const Object& p_obj)
{
    output << p_obj.ID << ' ';
    output << p_obj.measuredRelativePosition.x << ' ' << p_obj.measuredRelativePosition.y << ' ' << p_obj.measuredRelativePosition.z << ' ';
    output << p_obj.relativeMeasurementError.x << ' ' << p_obj.relativeMeasurementError.y << ' ' << p_obj.relativeMeasurementError.z << ' ';
    output << p_obj.estimatedRelativeLocation.x << ' ' << p_obj.estimatedRelativeLocation.y << ' ' << p_obj.estimatedRelativeLocation.z << ' ';
    output << p_obj.imagePositionAngle.x << ' ' << p_obj.imagePositionAngle.y << ' ';
    output << p_obj.imagePosition.x << ' ' << p_obj.imagePosition.y << ' ';
    output << p_obj.sizeOnScreen.x << ' ' << p_obj.sizeOnScreen.y << ' ';
    output << p_obj.timeLastSeen << ' ' << p_obj.timeSinceLastSeen << ' ' << p_obj.timeSeen << ' ' << p_obj.previousFrameTimestamp << ' ' << p_obj.isVisible << ' ';
    return output;
}

std::istream& operator>> (std::istream& input, Object& p_obj)
{
    input >> p_obj.ID;
    input >> p_obj.measuredRelativePosition.x >> p_obj.measuredRelativePosition.y >> p_obj.measuredRelativePosition.z;
    input >> p_obj.relativeMeasurementError.x >> p_obj.relativeMeasurementError.y >> p_obj.relativeMeasurementError.z;
    input >> p_obj.estimatedRelativeLocation.x >> p_obj.estimatedRelativeLocation.y >> p_obj.estimatedRelativeLocation.z;
    input >> p_obj.imagePositionAngle.x >> p_obj.imagePositionAngle.y;
    input >> p_obj.imagePosition.x >> p_obj.imagePosition.y;
    input >> p_obj.sizeOnScreen.x >> p_obj.sizeOnScreen.y;
    input >> p_obj.timeLastSeen >> p_obj.timeSinceLastSeen >> p_obj.timeSeen >> p_obj.previousFrameTimestamp >> p_obj.isVisible;
    return input;
}
