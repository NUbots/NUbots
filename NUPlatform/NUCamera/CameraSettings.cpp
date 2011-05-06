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
    p_hue = source.p_hue;
    p_redChroma = source.p_redChroma;
    p_blueChroma = source.p_blueChroma;
    p_gain = source.p_gain;
    p_exposure = source.p_exposure;
    p_autoExposure = source.p_autoExposure;
    p_autoWhiteBalance = source.p_autoWhiteBalance;
    p_autoGain = source.p_autoGain;
    
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
    p_hue.set(parameters[3], -180, 180, "the degree to which a stimulus can be described as similar to or different from stimuli that are described as red, green, blue, and yellow");
    p_redChroma.set(parameters[4], 0, 255, "the perceived intensity of a specific color (red)");
    p_blueChroma.set(parameters[5], 0, 255, "the perceived intensity of a specific color (blue)");
    p_exposure.set(parameters[6], 0, 510, "total amount of light allowed to fall on the photographic medium");
    p_gain.set(parameters[7], 0, 255, "ISO sensitivity");
    p_autoExposure.set(parameters[8], 0, 0, "permanently off");
    p_autoWhiteBalance.set(parameters[9], 0, 0, "permanently off");
    p_autoGain.set(parameters[10], 0, 0, "permanently off");
    
    copyParams();  
    
    return;                               
}

CameraSettings::CameraSettings(const std::vector<Parameter> parameters)
{
    SetDefaults();
    
    p_brightness = parameters[0];
    p_contrast = parameters[1];
    p_saturation = parameters[2];
    p_hue = parameters[3];
    p_redChroma = parameters[4];
    p_blueChroma = parameters[5];
    p_exposure = parameters[6];
    p_gain = parameters[7];
    p_autoExposure = parameters[8];
    p_autoWhiteBalance = parameters[9];
    p_autoGain = parameters[10];
    
    copyParams();
    
    return;
}

std::vector<float> CameraSettings::getAsVector()
{
    vector<float> parameters;
    
    parameters.push_back(p_brightness.get());
    parameters.push_back(p_contrast.get());
    parameters.push_back(p_saturation.get());
    parameters.push_back(p_hue.get());
    parameters.push_back(p_redChroma.get());
    parameters.push_back(p_blueChroma.get());
    parameters.push_back(p_exposure.get());
    parameters.push_back(p_gain.get());
    parameters.push_back(p_autoExposure.get());
    parameters.push_back(p_autoWhiteBalance.get());
    parameters.push_back(p_autoGain.get());
    
    copyParams();
    
    return parameters;   
}


std::vector<Parameter> CameraSettings::getAsParameters()
{
    vector<Parameter> parameters;
    
    parameters.push_back(p_brightness);
    parameters.push_back(p_contrast);
    parameters.push_back(p_saturation);
    parameters.push_back(p_hue);
    parameters.push_back(p_redChroma);
    parameters.push_back(p_blueChroma);
    parameters.push_back(p_exposure);
    parameters.push_back(p_gain);
    parameters.push_back(p_autoExposure);
    parameters.push_back(p_autoWhiteBalance);
    parameters.push_back(p_autoGain);
    
    copyParams();
    
    return parameters;
}


void CameraSettings::SetDefaults()
{    
    p_brightness.set(0, 0, 255, "an attribute of visual perception in which a source appears to be radiating or reflecting light");  
    p_contrast.set(0, 0, 127, "the difference in color and light between parts of an image");
    p_saturation.set(0, 0, 255, "the difference between a color against gray");
    p_hue.set(0, -180, 180, "the degree to which a stimulus can be described as similar to or different from stimuli that are described as red, green, blue, and yellow");
    p_redChroma.set(0, 0, 255, "the perceived intensity of a specific color (red)");
    p_blueChroma.set(0, 0, 255, "the perceived intensity of a specific color (blue)");
    p_exposure.set(0, 0, 510, "total amount of light allowed to fall on the photographic medium");
    p_gain.set(0, 0, 255, "ISO sensitivity");
    p_autoExposure.set(0, 0, 0, "permanently off");
    p_autoWhiteBalance.set(0, 0, 0, "permanently off");
    p_autoGain.set(0, 0, 0, "permanently off");
    
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
    while (not myStream.eof())
    {
          Parameter p;
          myStream >> p;
          if (p.name().size() != 0)
             parameters.push_back(p);
    }
    
    //debug << "HELLO WORLD" << endl;
    //debug << parameters << endl;
    //debug << "HELLO WORLD 2" << endl;
    
    p_brightness = parameters[0];
    p_contrast = parameters[1];
    p_saturation = parameters[2];
    p_hue = parameters[3];
    p_redChroma = parameters[4];
    p_blueChroma = parameters[5];
    p_exposure = parameters[6];
    p_gain = parameters[7];
    p_autoExposure = parameters[8];
    p_autoWhiteBalance = parameters[9];
    p_autoGain = parameters[10];
    
    copyParams();
    
    return;
}

void CameraSettings::copyParams()
{
     brightness = p_brightness.get();
     contrast = p_contrast.get();
     saturation = p_saturation.get();
     hue = p_hue.get();
     redChroma = p_redChroma.get();
     blueChroma = p_blueChroma.get();
     exposure = p_exposure.get();
     gain = p_gain.get();
     autoExposure = p_autoExposure.get();
     autoWhiteBalance = p_autoWhiteBalance.get();
     autoGain = p_autoGain.get();
     
     return;    
}


/*! @brief Put the entire contents of the CameraSettings class into a stream
 */
std::ostream& operator<< (std::ostream& output, const CameraSettings& p_cameraSetting)
{
    output << p_cameraSetting.p_gain.get() << " ";
    output << p_cameraSetting.p_exposure.get() << " ";
    output << p_cameraSetting.p_brightness.get() << " ";
    output << p_cameraSetting.p_contrast.get()  << " ";
    output << p_cameraSetting.p_saturation.get() << " ";
    output << p_cameraSetting.p_hue.get()  << " ";
    output << p_cameraSetting.p_redChroma.get() << " ";
    output << p_cameraSetting.p_blueChroma.get()  << " ";

    // Auto values
    output << p_cameraSetting.p_autoExposure.get() << " ";
    output << p_cameraSetting.p_autoWhiteBalance.get() << " ";
    output << p_cameraSetting.p_autoGain.get() << " ";

    output << p_cameraSetting.activeCamera << " ";

    return output;
}

/*! @brief Get the entire contents of the CameraSettings class from a stream
 */

std::istream& operator>> (std::istream& input, CameraSettings& p_cameraSetting)
{
    int temp;          
              
    input >> temp;    
    p_cameraSetting.p_gain.set(temp);
    input >> temp;
    p_cameraSetting.p_exposure.set(temp);
    input >> temp;
    p_cameraSetting.p_brightness.set(temp);
    input >> temp;
    p_cameraSetting.p_contrast.set(temp);
    input >> temp;
    p_cameraSetting.p_saturation.set(temp);
    input >> temp;
    p_cameraSetting.p_hue.set(temp);
    input >> temp;
    p_cameraSetting.p_redChroma.set(temp);
    input >> temp;
    p_cameraSetting.p_blueChroma.set(temp);
    input >> temp;
    p_cameraSetting.p_autoExposure.set(temp);
    input >> temp;
    p_cameraSetting.p_autoWhiteBalance.set(temp);
    input >> temp;
    p_cameraSetting.p_autoGain.set(temp); 
    
    

    int tempCamera;
    input >> tempCamera;
    p_cameraSetting.activeCamera = CameraSettings::Camera(tempCamera);
    p_cameraSetting.copyParams();
    
    return input;
}

