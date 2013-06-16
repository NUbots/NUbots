/*! @file SensorCalibration.h
    @brief Declaration of SensorCalibration class.

    @class SensorCalibration
    @brief Class used to store calibration information for the body orientation
    and the camera position.

    @author Steven Nicklin

    Copyright (c) 2013 Steven Nicklin

    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include "Tools/Math/Vector2.h"
#include "Tools/Math/Vector3.h"
#include "debug.h"
#include <fstream>
#include <sstream>


class SensorCalibration
{
public:
    // Constructors
    // Empty
    SensorCalibration(): m_body_angle_offset(), m_camera_angle_offset(), m_neck_position_offset(), m_body_position_offset(), m_camera_position_offset()
    {
        // Darwin default
        m_camera_position_offset.x = 3.32f;
        m_camera_position_offset.y = 0.f;
        m_camera_position_offset.z = 3.44f;
    }

    // From Vector values
    SensorCalibration(Vector2<double> body_angle, Vector3<double> camera_angle, Vector3<double> neck_position, Vector3<double> body_position, Vector3<double> camera_position):
        m_body_angle_offset(body_angle), m_camera_angle_offset(camera_angle), m_neck_position_offset(neck_position), m_body_position_offset(body_position), m_camera_position_offset(camera_position) {}

    // From file.
    SensorCalibration(const std::string& filename)
    {
        ReadSettings(filename);
        return;
    }

    // File I/O
    bool ReadSettings(const std::string& filename)
    {
        bool loaded = false;
        std::ifstream file(filename.c_str());

        if (file.is_open())
        {
            while (not file.eof())
            {
                std::string buffer;
                std::string setting_buffer;

                std::getline(file, buffer);
                std::stringstream ss(buffer);
                std::getline(ss, setting_buffer, ':');
                if(setting_buffer == "body angle offset")
                {
                    ss >> m_body_angle_offset;
                }
                else if(setting_buffer == "camera angle offset")
                {
                    ss >> m_camera_angle_offset;
                }
                else if(setting_buffer == "neck position offset")
                {
                    ss >> m_neck_position_offset;
                }
                else if(setting_buffer == "body position offset")
                {
                    ss >> m_body_position_offset;
                }
                else if(setting_buffer == "camera position offset")
                {
                    ss >> m_camera_position_offset;
                }
            }
            loaded = true;
        }
        else
        {
            errorlog << "SensorCalibration::LoadFromConfigFile(). Unable to read sensor calibration." << std::endl;
        }
        file.close();
        return loaded;
    }

    bool WriteSettings(const std::string& filename)
    {
        bool written = false;
        std::ofstream file(filename.c_str());

        if (file.is_open())
        {
            file << "body angle offset: " << m_body_angle_offset << std::endl;
            file << "camera angle offset: " << m_camera_angle_offset << std::endl;
            file << "neck position offset: " << m_neck_position_offset << std::endl;
            file << "body position offset: " << m_body_position_offset << std::endl;
            file << "camera position offset: " << m_camera_position_offset << std::endl;
            written = true;
        }
        else
        {
            errorlog << "SensorCalibration::LoadFromConfigFile(). Unable to write sensor calibration." << std::endl;
        }
        file.close();
        return written;
    }

    // Data
    Vector2<double> m_body_angle_offset;
    Vector3<double> m_camera_angle_offset;

    Vector3<double> m_neck_position_offset;

    Vector3<double> m_body_position_offset;
    Vector3<double> m_camera_position_offset;

};
