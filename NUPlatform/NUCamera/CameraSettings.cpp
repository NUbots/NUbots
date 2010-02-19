#include "CameraSettings.h"
#include "Tools/FileFormats/Parse.h"

CameraSettings::CameraSettings()
{
    SetDefaults();
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
    activeCamera = 1;

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
        saturation = configParser.GetAsInt("Hue");

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
        activeCamera = configParser.GetAsInt("ActiveCamera");

    return;
}
