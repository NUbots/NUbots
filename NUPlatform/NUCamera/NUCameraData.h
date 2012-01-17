/*!
@file CameraData.h 
@brief Declaration of NUbots CameraData class.
*/

#ifndef NUCAMERADATA_H
#define NUCAMERADATA_H

#include <string>

/*!
@brief Class used to store the properties and settings of a camera.

Used to store physical properties of the camera such as field of view 
and focal length information, as well as the unique ID for the camera for use in systems
with more than one camera available. The settings at which the image was taken may also be stored.
*/
class NUCameraData
{
public:
    /*!
      @brief default Constructor.
      */
    NUCameraData();
	 /*!
      @brief Constructor to load from specified file.
      */
    NUCameraData(const std::string& fileName);
    NUCameraData(const char* fileName);
    /*!
      @brief Copy constructor.
      */
    NUCameraData(const NUCameraData& sourceData);
    bool LoadFromConfigFile(const char* fileName);
    bool SetByName(const std::string& parameter, float value);

public:
        int m_cameraID;               //!< The unique ID that identifies the camera.
        float m_horizontalFov;        //!< The horizontal field of view of the camera.
        float m_verticalFov;          //!< The vertical field of view of the camera.
	//!\todo{Add any required camera settings to the image information}
};
#endif
