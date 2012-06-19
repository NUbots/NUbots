#include "CameraSettings.h"
#include "Tools/FileFormats/Parse.h"
#include "debug.h"
#include <fstream>

CameraSettings::CameraSettings()
{
    SetDefaults();
    return;
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
    p_hue = source.p_hue;
    p_redChroma = source.p_redChroma;
    p_blueChroma = source.p_blueChroma;
    p_autoExposure = source.p_autoExposure;
    p_autoGain = source.p_autoGain;
    p_powerLineFrequency = source.p_powerLineFrequency;
    p_whiteBalanceTemperature = source.p_whiteBalanceTemperature;
    p_sharpness = source.p_sharpness;
    p_exposureAuto = source.p_exposureAuto;
    p_exposureAbsolute = source.p_exposureAbsolute;
    p_exposureAutoPriority = source.p_exposureAutoPriority;

    p_valid = true;
    
    copyParams();

    // Extra
    activeCamera = source.activeCamera;
    return;
}

CameraSettings::CameraSettings(const std::string& configFileName)
{
    LoadFromFile(configFileName);
    return;
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
    p_hue.set(parameters[6], -180, 180, "the degree to which a stimulus can be described as similar to or different from stimuli that are described as red, green, blue, and yellow");
    p_redChroma.set(parameters[7], 0, 255, "the perceived intensity of a specific color (red)");
    p_blueChroma.set(parameters[8], 0, 255, "the perceived intensity of a specific color (blue)");
    p_autoExposure.set(parameters[9], 0, 0, "permanently off");
    p_autoGain.set(parameters[10], 0, 0, "permanently off");
    p_powerLineFrequency.set(parameters[11], 0, 2, "off, 50Hz and 60Hz");
    p_whiteBalanceTemperature.set(parameters[12], 0, 10000, "WKelvin value for colour temperature");
    p_sharpness.set(parameters[13], 0, 255, "");
    p_exposureAuto.set(parameters[14], 1, 1, "permanently off");
    p_exposureAbsolute.set(parameters[15], 0, 10000, "total amount of light allowed to fall on the photographic medium");
    p_exposureAutoPriority.set(parameters[16], 0, 0, "permanently off");

    p_valid = true;
    
    copyParams();  
    
    return;                               
}

CameraSettings::CameraSettings(const std::vector<Parameter> parameters)
{
    SetDefaults();

    for(unsigned int i=0; i<parameters.size(); i++) {
        if(!SetByName(parameters[i]))
            debug << "CameraSettings::LoadFromFile(): \"" << parameters[i] << "\" not found." << endl;
    }
    p_valid = true;
    
    copyParams();
    
    return;
}

std::vector<float> CameraSettings::getAsVector() const
{
    vector<float> parameters;
    
    parameters.push_back(p_brightness.get());
    parameters.push_back(p_contrast.get());
    parameters.push_back(p_saturation.get());
    parameters.push_back(p_gain.get());
    parameters.push_back(p_exposure.get());
    parameters.push_back(p_autoWhiteBalance.get());
    parameters.push_back(p_hue.get());
    parameters.push_back(p_redChroma.get());
    parameters.push_back(p_blueChroma.get());
    parameters.push_back(p_autoExposure.get());
    parameters.push_back(p_autoGain.get());
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
    vector<Parameter> parameters;
    
    parameters.push_back(p_brightness);
    parameters.push_back(p_contrast);
    parameters.push_back(p_saturation);
    parameters.push_back(p_gain);
    parameters.push_back(p_exposure);
    parameters.push_back(p_autoWhiteBalance);
    parameters.push_back(p_hue);
    parameters.push_back(p_redChroma);
    parameters.push_back(p_blueChroma);
    parameters.push_back(p_autoExposure);
    parameters.push_back(p_autoGain);
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
    p_hue.set(0, -180, 180, "the degree to which a stimulus can be described as similar to or different from stimuli that are described as red, green, blue, and yellow");
    p_redChroma.set(0, 0, 255, "the perceived intensity of a specific color (red)");
    p_blueChroma.set(0, 0, 255, "the perceived intensity of a specific color (blue)");
    p_autoExposure.set(0, 0, 0, "permanently off");
    p_autoGain.set(0, 0, 0, "permanently off");
    p_powerLineFrequency.set(1, 0, 2, "off, 50Hz and 60Hz");
    p_whiteBalanceTemperature.set(0, 0, 10000, "Kelvin value for colour temperature");
    p_sharpness.set(0, 0, 255, "");
    p_exposureAuto.set(1, 1, 1, "permanently off");
    p_exposureAbsolute.set(0, 0, 10000, "total amount of light allowed to fall on the photographic medium");
    p_exposureAutoPriority.set(0, 0, 0, "permanently off");
    
    copyParams();

    // Extra
    activeCamera = BOTTOM_CAMERA;

    return;
}

void CameraSettings::LoadFromFile(const std::string& configFileName)
{     
    fstream myStream(configFileName.c_str());
    vector<Parameter> parameters;   
     
    SetDefaults();  // Set default values incase some are missing in the file.
    if (myStream.is_open())
    {
		while (not myStream.eof())
		{
			  Parameter p;
			  myStream >> p;
			  if (p.name().size() != 0)
				 parameters.push_back(p);
		}

        for(unsigned int i=0; i<parameters.size(); i++) {
            if(!SetByName(parameters[i]))
                debug << "CameraSettings::LoadFromFile(): \"" << parameters[i].name() << "\" not found." << endl;
            else
                debug << "CameraSettings::LoadFromFile(): " << parameters[i].name() << " " << parameters[i].get() <<
                         " [" << parameters[i].min() << ", " << parameters[i].max() << "]" << endl;
        }
        p_valid = true;
    }
    else
    	debug << "CameraSettings::LoadFromFile() Failed to load CameraSettings from " << configFileName << endl;
    
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
    else if(p.compareName("AutoGain"))
        p_autoGain = p;
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
    else if(p.compareName("Hue"))
        p_hue = p;
    else if(p.compareName("RedChroma"))
        p_redChroma = p;
    else if(p.compareName("BlueChroma"))
        p_blueChroma = p;
    else if(p.compareName("Exposure"))
        p_exposure = p;
    else if(p.compareName("AutoExposure"))
        p_autoExposure = p;
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
    hue = p_hue.get();
    redChroma = p_redChroma.get();
    blueChroma = p_blueChroma.get();
    autoExposure = p_autoExposure.get();
    autoGain = p_autoGain.get();
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
    vector<float> vals = p_cameraSetting.getAsVector();
    for(unsigned int i=0; i<vals.size(); i++)
        output << vals.at(i) << " ";

    output << p_cameraSetting.p_valid << " ";

    output << p_cameraSetting.activeCamera << " ";

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
    p_cameraSetting.p_hue.set(temp);
    input >> temp;
    p_cameraSetting.p_redChroma.set(temp);
    input >> temp;
    p_cameraSetting.p_blueChroma.set(temp);
    input >> temp;
    p_cameraSetting.p_autoExposure.set(temp);
    input >> temp;
    p_cameraSetting.p_autoGain.set(temp);
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
    
    int tempCamera;
    input >> tempCamera;
    p_cameraSetting.activeCamera = CameraSettings::Camera(tempCamera);
    p_cameraSetting.copyParams();
    
    return input;
}

