#ifndef CAMERASETTINGS_H
#define CAMERASETTINGS_H

#include <string>

class CameraSettings
{
    public:
        int brightness;
        int contrast;
        int saturation;
        int hue;
        int redChroma;
        int blueChroma;
        int gain;
        int exposure;

        int autoExposure;
        int autoWhiteBalance;
        int autoGain;

        int activeCamera;

        CameraSettings();
        CameraSettings(const std::string& configFileName);
};
#endif

