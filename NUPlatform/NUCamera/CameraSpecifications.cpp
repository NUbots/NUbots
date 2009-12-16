#include "CameraSpecifications.h"
#include "Tools/FileFormats/Parse.h"

CameraSpecifications::CameraSpecifications()
{
}

CameraSpecifications::CameraSpecifications(const char* fileName)
{
    LoadFromConfigFile(fileName);
    return;
}

CameraSpecifications::CameraSpecifications(const CameraSpecifications& sourceSpec)
{
    this->resolutionWidth = sourceSpec.resolutionWidth;
    this->resolutionHeight = sourceSpec.resolutionHeight;
    this->fps = sourceSpec.fps;
    this->horizontalFov = sourceSpec.horizontalFov;
    this->verticalFov = sourceSpec.verticalFov;
    this->focalLength = sourceSpec.focalLength;
    return;
}

bool CameraSpecifications::LoadFromConfigFile(const char* fileName)
{
    Parse file;
    bool fileParsed = file.ParseFile(fileName);
    if(fileParsed == false) return false;
    if(file.HasKey("Resolution Width"))
    {
        this->resolutionWidth = file.GetAsInt("Resolution Width");
    }
    else
    {
        this->resolutionWidth = 0;
    }
    if(file.HasKey("Resolution Height"))
    {
        this->resolutionHeight = file.GetAsInt("Resolution Height");
    }
    else
    {
        this->resolutionHeight = 0;
    }

    if(file.HasKey("Fps"))
    {
        this->fps = file.GetAsInt("Fps");
    }
    else
    {
        this->fps = 0;
    }
    if(file.HasKey("Horizontal Fov"))
    {
        this->horizontalFov = file.GetAsDouble("Horizontal Fov");
    }
    else
    {
        this->horizontalFov = 0.0;
    }
    if(file.HasKey("Vertical Fov"))
    {
        this->verticalFov = file.GetAsDouble("Vertical Fov");
    }
    else
    {
        this->verticalFov = 0.0;
    }
    if(file.HasKey("Focal Length"))
    {
        this->focalLength = file.GetAsDouble("Focal Length");
    }
    else
    {
        this->verticalFov = 0.0;
    }
    return fileParsed;
}
