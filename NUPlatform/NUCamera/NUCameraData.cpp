#include "NUCameraData.h"
#include <istream>
#include <sstream>
#include "debug.h"
#include "Tools/Math/General.h"

NUCameraData::NUCameraData()
{
}

NUCameraData::NUCameraData(const std::string& fileName)
{
    LoadFromConfigFile(fileName.c_str());
    return;
}

NUCameraData::NUCameraData(const char* fileName)
{
    LoadFromConfigFile(fileName);
    return;
}

NUCameraData::NUCameraData(const NUCameraData& sourceData)
{
    this->m_cameraID = sourceData.m_cameraID;
    this->m_horizontalFov = sourceData.m_horizontalFov;
    this->m_verticalFov = sourceData.m_verticalFov;
    return;
}

bool NUCameraData::SetByName(const std::string& parameter, float value)
{
    bool set = false;
    if (parameter == "horizontal fov")
    {
        this->m_horizontalFov = mathGeneral::deg2rad(value);
        set = true;
    }
    else if (parameter == "vertical fov")
    {
        this->m_verticalFov = mathGeneral::deg2rad(value);
        set = true;
    }
//    if(set)
//    {
//        std::cout << parameter << ": " << value << std::endl;
//    }
    return set;
}

bool NUCameraData::LoadFromConfigFile(const char* fileName)
{
    bool loaded = false;
    std::ifstream file(fileName);
    if (file.is_open())
    {
        while (not file.eof())
        {
            std::string buffer;
            std::string setting_buffer;
            float value_buffer;

            std::getline(file, buffer);
            std::stringstream ss(buffer);
            std::getline(ss, setting_buffer, ':');
            ss >> value_buffer;
            SetByName(setting_buffer, value_buffer);
        }
        loaded = true;
    }
    else
    {
        errorlog << "NUCameraData::LoadFromConfigFile(). Unable to load camera specifications." << std::endl;
    }
    file.close();
    return loaded;
}
