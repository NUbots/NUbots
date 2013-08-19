#include "CameraSettings.h"
#include <fstream>

namespace messages
{

    CameraSettings::CameraSettings()
    {
        SetDefaults();
    }

    CameraSettings::CameraSettings(const CameraSettings& source)
    {
        // Image settings
        p_brightness = source.p_brightness;
        p_contrast = source.p_contrast;
        p_saturation = source.p_saturation;
        p_gain = source.p_gain;
        p_exposure = source.p_exposure;
        p_autoWhiteBalance = source.p_autoWhiteBalance;
        p_powerLineFrequency = source.p_powerLineFrequency;
        p_whiteBalanceTemperature = source.p_whiteBalanceTemperature;
        p_sharpness = source.p_sharpness;
        p_exposureAuto = source.p_exposureAuto;
        p_exposureAbsolute = source.p_exposureAbsolute;
        p_exposureAutoPriority = source.p_exposureAutoPriority;

        p_valid = true;
    
        copyParams();
    }

    CameraSettings::CameraSettings(const std::string& configFileName)
    {
        LoadFromFile(configFileName);
    }

    CameraSettings::CameraSettings(const std::vector<float> parameters)
    {
        SetDefaults();    
    
        p_brightness.set(parameters[0], 0, 255, "an attribute of visual perception in which a source appears to be radiating or reflecting light");
        p_contrast.set(parameters[1], 0, 127, "the difference in color and light between parts of an image");
        p_saturation.set(parameters[2], 0, 255, "the difference between a color against gray");
        p_gain.set(parameters[3], 0, 255, "ISO sensitivity");
        p_exposure.set(parameters[4], 0, 510, "total amount of light allowed to fall on the photographic medium");
        p_autoWhiteBalance.set(parameters[5], 0, 0, "permanently off");
        p_powerLineFrequency.set(parameters[6], 0, 2, "off, 50Hz and 60Hz");
        p_whiteBalanceTemperature.set(parameters[7], 0, 10000, "WKelvin value for colour temperature");
        p_sharpness.set(parameters[8], 0, 255, "");
        p_exposureAuto.set(parameters[9], 1, 1, "permanently off");
        p_exposureAbsolute.set(parameters[10], 0, 10000, "total amount of light allowed to fall on the photographic medium");
        p_exposureAutoPriority.set(parameters[11], 0, 0, "permanently off");

        p_valid = true;
    
        copyParams();  
    
        return;                               
    }

    CameraSettings::CameraSettings(const std::vector<Parameter> parameters)
    {
        SetDefaults();

        for(unsigned int i=0; i<parameters.size(); i++) {
            SetByName(parameters[i]);
        }
        p_valid = true;
    
        copyParams();
    
        return;
    }

    std::vector<float> CameraSettings::getAsVector() const
    {
        std::vector<float> parameters;
    
        parameters.push_back(p_brightness.get());
        parameters.push_back(p_contrast.get());
        parameters.push_back(p_saturation.get());
        parameters.push_back(p_gain.get());
        parameters.push_back(p_exposure.get());
        parameters.push_back(p_autoWhiteBalance.get());
        parameters.push_back(p_powerLineFrequency.get());
        parameters.push_back(p_whiteBalanceTemperature.get());
        parameters.push_back(p_sharpness.get());
        parameters.push_back(p_exposureAuto.get());
        parameters.push_back(p_exposureAbsolute.get());
        parameters.push_back(p_exposureAutoPriority.get());
    
        return parameters;   
    }


    std::vector<Parameter> CameraSettings::getAsParameters()
    {
        std::vector<Parameter> parameters;
    
        parameters.push_back(p_brightness);
        parameters.push_back(p_contrast);
        parameters.push_back(p_saturation);
        parameters.push_back(p_gain);
        parameters.push_back(p_exposure);
        parameters.push_back(p_autoWhiteBalance);
        parameters.push_back(p_powerLineFrequency);
        parameters.push_back(p_whiteBalanceTemperature);
        parameters.push_back(p_sharpness);
        parameters.push_back(p_exposureAuto);
        parameters.push_back(p_exposureAbsolute);
        parameters.push_back(p_exposureAutoPriority);
    
        return parameters;
    }


    void CameraSettings::SetDefaults()
    {    
        p_valid = false;
        p_brightness.set(0, 0, 255, "an attribute of visual perception in which a source appears to be radiating or reflecting light");
        p_contrast.set(0, 0, 127, "the difference in color and light between parts of an image");
        p_saturation.set(0, 0, 255, "the difference between a color against gray");
        p_gain.set(0, 0, 255, "ISO sensitivity");
        p_exposure.set(0, 0, 510, "total amount of light allowed to fall on the photographic medium");
        p_autoWhiteBalance.set(0, 1, 1, "permanently off");
        p_powerLineFrequency.set(1, 0, 2, "off, 50Hz and 60Hz");
        p_whiteBalanceTemperature.set(0, 0, 10000, "Kelvin value for colour temperature");
        p_sharpness.set(0, 0, 255, "");
        p_exposureAuto.set(1, 1, 1, "permanently off");
        p_exposureAbsolute.set(0, 0, 10000, "total amount of light allowed to fall on the photographic medium");
        p_exposureAutoPriority.set(0, 0, 0, "permanently off");
    
        copyParams();
    }

    void CameraSettings::LoadFromFile(const std::string& configFileName)
    {     
        std::fstream myStream(configFileName.c_str());
     
        SetDefaults();  // Set default values incase some are missing in the file.
        if (myStream.is_open())
        {
            while (!myStream.eof())
            {
                Parameter p;
                myStream >> p;
                if (p.name().size() != 0)
                    SetByName(p);
            }

            p_valid = true;
        }
    
        copyParams();
    
        return;
    }

    bool CameraSettings::SetByName(const Parameter& p)
    {
        if(p.compareName("Brightness"))
            p_brightness = p;
        else if(p.compareName("Contrast"))
            p_contrast = p;
        else if(p.compareName("Saturation"))
            p_saturation = p;
        else if(p.compareName("Gain"))
            p_gain = p;
        else if(p.compareName("AutoWhiteBalance"))
            p_autoWhiteBalance = p;
        else if(p.compareName("PowerLineFrequency"))
            p_powerLineFrequency = p;
        else if(p.compareName("WhiteBalanceTemperature"))
            p_whiteBalanceTemperature = p;
        else if(p.compareName("Sharpness"))
            p_sharpness = p;
        else if(p.compareName("ExposureAbsolute"))
            p_exposureAbsolute = p;
        else if(p.compareName("ExposureAuto"))
            p_exposureAuto = p;
        else if(p.compareName("ExposureAutoPriority"))
            p_exposureAutoPriority = p;
        else if(p.compareName("Exposure"))
            p_exposure = p;
        else
            return false; //setting not found
        return true;
    }

    void CameraSettings::copyParams()
    {
        brightness = p_brightness.get();
        contrast = p_contrast.get();
        saturation = p_saturation.get();
        gain = p_gain.get();
        exposure = p_exposure.get();
        autoWhiteBalance = p_autoWhiteBalance.get();
        powerLineFrequency = p_powerLineFrequency.get();
        whiteBalanceTemperature = p_whiteBalanceTemperature.get();
        sharpness = p_sharpness.get();
        exposureAuto = p_exposureAuto.get();
        exposureAbsolute = p_exposureAbsolute.get();
        exposureAutoPriority = p_exposureAutoPriority.get();
    }


    /*! @brief Put the entire contents of the CameraSettings class into a stream
     */
    std::ostream& operator<< (std::ostream& output, const CameraSettings& p_cameraSetting)
    {
        std::vector<float> vals = p_cameraSetting.getAsVector();
        for(unsigned int i=0; i<vals.size(); i++)
            output << vals.at(i) << " ";

        output << p_cameraSetting.p_valid << " ";

        return output;
    }

    /*! @brief Get the entire contents of the CameraSettings class from a stream
     */

    std::istream& operator>> (std::istream& input, CameraSettings& p_cameraSetting)
    {
        int temp;

        input >> temp;
        p_cameraSetting.p_brightness.set(temp);
        input >> temp;
        p_cameraSetting.p_contrast.set(temp);
        input >> temp;
        p_cameraSetting.p_saturation.set(temp);
        input >> temp;
        p_cameraSetting.p_gain.set(temp);
        input >> temp;
        p_cameraSetting.p_exposure.set(temp);
        input >> temp;
        p_cameraSetting.p_autoWhiteBalance.set(temp);
        input >> temp;
        p_cameraSetting.p_powerLineFrequency.set(temp);
        input >> temp;
        p_cameraSetting.p_whiteBalanceTemperature.set(temp);
        input >> temp;
        p_cameraSetting.p_sharpness.set(temp);
        input >> temp;
        p_cameraSetting.p_exposureAuto.set(temp);
        input >> temp;
        p_cameraSetting.p_exposureAbsolute.set(temp);
        input >> temp;
        p_cameraSetting.p_exposureAutoPriority.set(temp);
        input >> temp;
        p_cameraSetting.p_valid = temp;
    
        p_cameraSetting.copyParams();
    
        return input;
    }

}
