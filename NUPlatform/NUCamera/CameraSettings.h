/*!
@file CameraSettings.h
@brief Declaration of CameraSettings container class.
@author Steven Nicklin
*/

#ifndef CAMERASETTINGS_H
#define CAMERASETTINGS_H

#include <string>
#include <vector>
#include <iostream>
#include "Tools/Optimisation/Parameter.h"

/*!
    @brief Class used to store camera settings.
  */
class CameraSettings
{
    public:
        enum Camera
        {
            UNKNOWN_CAMERA = 0x00,
            TOP_CAMERA = 0x01,
            BOTTOM_CAMERA = 0x02,
            NUM_CAMERAS = 2
        };

        static std::string cameraName(Camera theCamera)
        {
            std::string camera_name;
            switch(theCamera)
            {
                case TOP_CAMERA:
                    camera_name = "Top Camera";
                    break;
                case BOTTOM_CAMERA:
                    camera_name = "Bottom Camera";
                    break;
                default:
                    camera_name = "Unknown Camera";
            }
            return camera_name;
        }

        
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
        
        
        Camera activeCamera;
        
        Parameter p_brightness;         //!< Brightness setting.
        Parameter p_contrast;           //!< Contrast setting.
        Parameter p_saturation;         //!< Saturation setting.
        Parameter p_hue;                //!< Hue setting.
        Parameter p_redChroma;          //!< Red Chroma setting.
        Parameter p_blueChroma;         //!< Blue Chroma setting.
        Parameter p_gain;               //!< Gain setting.
        Parameter p_exposure;           //!< Exposure setting

        Parameter p_autoExposure;       //!< Auto Exposure setting.
        Parameter p_autoWhiteBalance;   //!< Auto White Balance setting.
        Parameter p_autoGain;           //!< Auto Gain setting.
        

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
        CameraSettings(const std::vector<float> parameters);
        
        /*!
            @brief Sets all of the settings to values given by parameters vector.
        */
        
        CameraSettings(const std::vector<Parameter> parameters);
        
        /*!
            @brief Sets all of the settings to values given by parameters vector.
        */
        
        void SetDefaults();

        /*!
            @brief Sets the settings to those found within the specified file.
            If a setting is not found, the default value is used.
        */
        
        void copyParams();
        
        void LoadFromFile(const std::string& configFileName);
        
        std::vector<float> getAsVector();
        
        /*!
            @brief Get camera settings and return as a vector of parameter values
        */        
        std::vector<Parameter> getAsParameters();
        
        /*!
            @brief Get camera settings and return as a vector of parameters
        */        

        friend std::ostream& operator<< (std::ostream& output, const CameraSettings& p_cameraSetting);
        friend std::istream& operator>> (std::istream& input, CameraSettings& p_cameraSetting);


};
#endif

