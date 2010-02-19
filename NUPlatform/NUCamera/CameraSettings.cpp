#include "CameraSettings.h"

CameraSettings::CameraSettings()
{
// Image settings
    brightness = 0;
    contrast = 0;
    saturation;
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
    activeCamera = 0;
}

CameraSettings::CameraSettings(const std::string& configFileName)
{
    CameraSettings();
}
