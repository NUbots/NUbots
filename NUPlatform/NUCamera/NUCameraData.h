/*!
@file CameraInfo.h 
@brief Declaration of NUbots CameraInfo class.
*/

#ifndef CAMERAINFO_H
#define CAMERAINFO_H

/*!
@brief Class used to store the properties and settings of a camera.

Used to store physical properties of the camera such as field of view 
and focal length information, as well as the unique ID for the camera for use in systems
with more than one camera available. The settings at which the image was taken may also be stored.
*/
class CameraInfo
{
public:
        int cameraID;               //!< The unique ID that identifies the camera.
        float horizontalFOV;        //!< The horizontal field of view of the camera.
        float verticalFOV;          //!< The vertical field of view of the camera.
        float focalLength;          //!< The focal length of the camera.
        float fps;                  //!< The Frames Per Second (FPS) at which the camera was running.

	//!\todo{Add any required camera settings to the image information}
};
#endif
