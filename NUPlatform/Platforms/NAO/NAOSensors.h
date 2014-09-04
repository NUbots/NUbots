/*! @file NAOSensors.h
    @brief Declaration of NAOsensors class

    @author Jason Kulk
 
    @class NAOSensors
    @brief A NAO sensors
 
    @author Jason Kulk
 
  Copyright (c) 2009 Jason Kulk
 
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

#ifndef NAOSENSORS_H
#define NAOSENSORS_H

#include "NUPlatform/NUSensors.h"
#include "Infrastructure/NUData.h"

#include <almemoryfastaccess/almemoryfastaccess.h>
using namespace AL;

#include <vector>
#include <string>


class NAOSensors : public NUSensors
{
public:
    NAOSensors();
    ~NAOSensors();
private:
    void getSensorsFromALMemory();
    void initBuffers();
    
    void copyFromHardwareCommunications();
    void copyFromJoints();
    void copyFromAccelerometerAndGyro();
    void copyFromDistance();
    void copyFromFeet();
    void copyFromButtons();
    void copyFromBattery();
    
private:
    static std::vector<std::string> m_jointposition_names;            //!< a std::list of device names for the joint position sensors
    static std::vector<std::string> m_jointtarget_names;              //!< a std::list of device names for the joint target sensors
    static std::vector<std::string> m_jointstiffness_names;           //!< a std::list of device names for the joint stiffness sensors
    static std::vector<std::string> m_jointcurrent_names;             //!< a std::list of device names for the joint current sensors
    static std::vector<std::string> m_jointtemperature_names;         //!< a std::list of device names for the joint temperature sensors
    static std::vector<std::string> m_accel_names;                    //!< a std::list of device names for the accelerometers
    static std::vector<std::string> m_gyro_names;                     //!< a std::list of device names for the gyrometers
    static std::vector<std::string> m_orientation_names;              //!< a std::list of device names for the orientation sensors
    static std::vector<std::string> m_foot_left_sole_names;           //!< a std::list of device names for the foot pressure sensors
    static std::vector<std::string> m_foot_right_sole_names;          //!< a std::list of device names for the foot pressure sensors
    static std::vector<std::string> m_foot_bumper_names;              //!< a std::list of device names for the foot touch sensors
    static std::vector<std::string> m_button_names;                   //!< a std::list of device names for the buttons
    static std::vector<std::string> m_battery_names;                  //!< a std::list of device names for the battery
    static std::vector<std::string> m_ultrasonic_left_distances;      //!< a std::list of device names for the ultrasonic distances
    static std::vector<std::string> m_ultrasonic_right_distances;     //!< a std::list of device names for the ultrasonic distances
    
    std::vector<NUData::id_t*> m_joint_ids;						//!< a std::vector containing pointers to all of the joint id_t. This is used to loop through all of the joints quickly
    
    std::vector<float> m_buffer_positions;						//!< temporary buffers to hold raw data taken from ALMemory (joint positions)
    std::vector<float> m_buffer_targets;							//!< temporary buffers to hold raw data taken from ALMemory (joint targets)
    std::vector<float> m_buffer_stiffnesses;						//!< temporary buffers to hold raw data taken from ALMemory (joint stiffnesses)
    std::vector<float> m_buffer_currents;						//!< temporary buffers to hold raw data taken from ALMemory (joint currents)
    std::vector<float> m_buffer_temperatures;					//!< temporary buffers to hold raw data taken from ALMemory (joint temperatures)
    std::vector<float> m_buffer_accelerometer;					//!< temporary buffers to hold raw data taken from ALMemory (accelerometer)
    std::vector<float> m_buffer_gyrometer;						//!< temporary buffers to hold raw data taken from ALMemory (gyrometer)
    std::vector<float> m_buffer_orientation;						//!< temporary buffers to hold raw data taken from ALMemory (orientation)
	std::vector<float> m_buffer_left_echos;						//!< temporary buffers to hold raw data taken from ALMemory (left ultrasonics)
    std::vector<float> m_buffer_right_echos;						//!< temporary buffers to hold raw data taken from ALMemory (right ultrasonics)
    std::vector<float> m_buffer_foot_lfsr;
    std::vector<float> m_buffer_foot_rfsr;
    std::vector<float> m_buffer_foot_bumper;	
    std::vector<float> m_buffer_buttons;
    std::vector<float> m_buffer_battery;
    
    std::vector<float> m_previous_positions;
    std::vector<float> m_previous_velocities;

    ALMemoryFastAccess* m_al_positions_access;              //!< fast access to almemory position access
    ALMemoryFastAccess* m_al_targets_access;                //!< fast access to almemory target access
    ALMemoryFastAccess* m_al_stiffness_access;              //!< fast access to almemory stiffness access
    ALMemoryFastAccess* m_al_current_access;                //!< fast access to almemory current access
    ALMemoryFastAccess* m_al_temperature_access;            //!< fast access to almemory temperature access
    ALMemoryFastAccess* m_al_accel_access;                  //!< fast access to almemory accelerometer access
    ALMemoryFastAccess* m_al_gyro_access;                   //!< fast access to almemory gyro access
    ALMemoryFastAccess* m_al_orientation_access;            //!< fast access to almemory orientation access
    ALMemoryFastAccess* m_al_footleftsole_access;           //!< fast access to almemory foot sole access
    ALMemoryFastAccess* m_al_footrightsole_access;          //!< fast access to almemory foot sole access
    ALMemoryFastAccess* m_al_footbumper_access;             //!< fast access to almemory foot bumper access
    ALMemoryFastAccess* m_al_button_access;                 //!< fast access to almemory button access
    ALMemoryFastAccess* m_al_battery_access;                //!< fast access to almemory battery access
    
    ALMemoryFastAccess* m_al_ultrasonic_left_distances;     //!< fast access to almemory left ultrasonic distance
    ALMemoryFastAccess* m_al_ultrasonic_right_distances;    //!< fast access to almemory right ultrasonic distance
};

#endif

