#include "CameraSettings.h"
#include "Tools/FileFormats/Parse.h"

CameraSettings::CameraSettings()
{
    SetDefaults();
    return;
}

CameraSettings::CameraSettings(const CameraSettings& source)
{
    // Image settings
    brightness = source.brightness;
    contrast = source.contrast;
    saturation = source.saturation;
    hue = source.hue;
    redChroma = source.redChroma;
    blueChroma = source.blueChroma;
    gain = source.gain;
    exposure = source.exposure;

    // Auto values
    autoExposure = source.autoExposure;
    autoWhiteBalance = source.autoWhiteBalance;
    autoGain = source.autoGain;

    // Extra
    activeCamera = source.activeCamera;
    return;
}

CameraSettings::CameraSettings(const std::string& configFileName)
{
    LoadFromFile(configFileName);
    return;
}

void CameraSettings::SetDefaults()
{
    // Image settings
    brightness = 0;
    contrast = 0;
    saturation = 0;
    hue = 0;
    redChroma = 0;
    blueChroma = 0;
    gain = 0;
    exposure = 0;

    // Auto values
    autoExposure = 0;
    autoWhiteBalance = 0;
    autoGain = 0;

    // Extra
    activeCamera = BOTTOM_CAMERA;

    return;
}

void CameraSettings::LoadFromFile(const std::string& configFileName)
{
    SetDefaults();  // Set default values incase some are missing in the file.

    // Setup parser
    Parse configParser;
    configParser.ParseFile(configFileName.c_str());

    // Brightness
    if(configParser.HasKey("Brightness"))
        brightness = configParser.GetAsInt("Brightness");

    // Contrast
    if(configParser.HasKey("Contrast"))
        contrast = configParser.GetAsInt("Contrast");

    // Saturation
    if(configParser.HasKey("Saturation"))
        saturation = configParser.GetAsInt("Saturation");

    // Hue
    if(configParser.HasKey("Hue"))
        hue = configParser.GetAsInt("Hue");

    // RedChroma
    if(configParser.HasKey("RedChroma"))
        redChroma = configParser.GetAsInt("RedChroma");

    // BlueChroma
    if(configParser.HasKey("BlueChroma"))
        blueChroma = configParser.GetAsInt("BlueChroma");

    // Gain
    if(configParser.HasKey("Gain"))
        gain = configParser.GetAsInt("Gain");

    // Exposure
    if(configParser.HasKey("Exposure"))
        exposure = configParser.GetAsInt("Exposure");

    // AutoExposure
    if(configParser.HasKey("AutoExposure"))
        autoExposure = configParser.GetAsInt("AutoExposure");

    // AutoWhiteBalance
    if(configParser.HasKey("AutoWhiteBalance"))
        autoWhiteBalance = configParser.GetAsInt("AutoWhiteBalance");

    // AutoGain
    if(configParser.HasKey("AutoGain"))
        autoGain = configParser.GetAsInt("AutoGain");

    // ActiveCamera
    if(configParser.HasKey("ActiveCamera"))
        activeCamera = Camera(configParser.GetAsInt("ActiveCamera"));

    return;
}


/*! @brief Put the entire contents of the CameraSettings class into a stream
 */
std::ostream& operator<< (std::ostream& output, const CameraSettings& p_cameraSetting)
{
    output << p_cameraSetting.gain << " ";
    output << p_cameraSetting.exposure << " ";
    output << p_cameraSetting.brightness << " ";
    output << p_cameraSetting.contrast  << " ";
    output << p_cameraSetting.saturation << " ";
    output << p_cameraSetting.hue  << " ";
    output << p_cameraSetting.redChroma  << " ";
    output << p_cameraSetting.blueChroma  << " ";

    // Auto values
    output << p_cameraSetting.autoExposure << " ";
    output << p_cameraSetting.autoWhiteBalance << " ";
    output << p_cameraSetting.autoGain << " ";

    output << p_cameraSetting.activeCamera << " ";

    return output;
}

/*! @brief Get the entire contents of the CameraSettings class from a stream
 */

std::istream& operator>> (std::istream& input, CameraSettings& p_cameraSetting)
{
    input >> p_cameraSetting.gain;
    input >> p_cameraSetting.exposure;
    input >> p_cameraSetting.brightness;
    input >> p_cameraSetting.contrast;
    input >> p_cameraSetting.saturation;
    input >> p_cameraSetting.hue;
    input >> p_cameraSetting.redChroma;
    input >> p_cameraSetting.blueChroma;

    // Auto values
    input >> p_cameraSetting.autoExposure;
    input >> p_cameraSetting.autoWhiteBalance;
    input >> p_cameraSetting.autoGain;

    input >> p_cameraSetting.activeCamera;

    return input;
}
