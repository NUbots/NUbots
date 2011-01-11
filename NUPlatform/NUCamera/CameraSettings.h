/*!
@file CameraSettings.h
@brief Declaration of CameraSettings container class.
@author Steven Nicklin
*/

#ifndef CAMERASETTINGS_H
#define CAMERASETTINGS_H

#include <string>
#include <iostream>

/*!
    @brief Class used to store camera settings.
  */
class CameraSettings
{
    public:
		enum Camera
		{
			TOP_CAMERA = 0x01,
			BOTTOM_CAMERA = 0x02
		};
        int brightness;         //!< Brightness setting.
        int contrast;           //!< Contrast setting.
        int saturation;         //!< Saturation setting.
        int hue;                //!< Hue setting.
        int redChroma;          //!< Red Chroma setting.
        int blueChroma;         //!< Blue Chroma setting.
        int gain;               //!< Gain setting.
        int exposure;           //!< Exposure setting

        int autoExposure;       //!< Auto Exposure setting.
        int autoWhiteBalance;   //!< Auto White Balance setting.
        int autoGain;           //!< Auto Gain setting.

        Camera activeCamera;       //!< The Active Camera.

        /*!
            @brief Default Constructor. Loads the default camera settings.
        */
        CameraSettings();

        /*!
            @brief Copy Constructor. Creates with identical settings to the given CameraSettings.
        */
        CameraSettings(const CameraSettings& source);

        /*!
            @brief Constructor with file path. Loads the camera settings from the specified file.
            @param configFileName The path of the configuration file.
        */
        CameraSettings(const std::string& configFileName);

        /*!
            @brief Sets all of the settings to their default values.
        */
        void SetDefaults();

        /*!
            @brief Sets the settings to those found within the specified file.
            If a setting is not found, the default value is used.
        */
        void LoadFromFile(const std::string& configFileName);

        friend std::ostream& operator<< (std::ostream& output, const CameraSettings& p_cameraSetting);
        friend std::istream& operator>> (std::istream& input, CameraSettings& p_cameraSetting);


};
#endif

