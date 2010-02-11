/*!
@file CameraData.h 
@brief Declaration of NUbots CameraData class.
*/

#ifndef NUCAMERADATA_H
#define NUCAMERADATA_H

/*!
@brief Class used to store the properties and settings of a camera.

Used to store physical properties of the camera such as field of view 
and focal length information, as well as the unique ID for the camera for use in systems
with more than one camera available. The settings at which the image was taken may also be stored.
*/
class CameraData
{
public:
    /*!
      @brief default Constructor.
      */
    CameraData();
	 /*!
      @brief Constructor to load from specified file.
      */
    CameraData(const char* fileName);
    /*!
      @brief Copy constructor.
      */
    CameraData(const CameraData& sourceData);
    bool LoadFromConfigFile(const char* fileName);

public:
        int cameraID;               //!< The unique ID that identifies the camera.
		int imageWidth;				//!< The horizontal resolution of the camera.
		int imageHeight;		    //!< The vertical resolution of the camera.
        float horizontalFOV;        //!< The horizontal field of view of the camera.
        float verticalFOV;          //!< The vertical field of view of the camera.
        float focalLength;          //!< The focal length of the camera.
        float fps;                  //!< The Frames Per Second (FPS) at which the camera was running.

	//!\todo{Add any required camera settings to the image information}
};
#endif
