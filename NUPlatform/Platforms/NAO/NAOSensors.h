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
#include "NUNAO.h"

class NAOSensors : public NUSensors
{
public:
    NAOSensors();
    ~NAOSensors();
private:
    void getSensorsFromALMemory();
    void copyFromHardwareCommunications();
    
private:
    static vector<string> m_jointposition_names;            //!< a list of device names for the joint position sensors
    static vector<string> m_jointtarget_names;              //!< a list of device names for the joint target sensors
    static vector<string> m_jointstiffness_names;           //!< a list of device names for the joint stiffness sensors
    static vector<string> m_jointcurrent_names;             //!< a list of device names for the joint current sensors
    static vector<string> m_jointtemperature_names;         //!< a list of device names for the joint temperature sensors
    static vector<string> m_accel_names;                    //!< a list of device names for the accelerometers
    static vector<string> m_gyro_names;                     //!< a list of device names for the gyrometers
    static vector<string> m_foot_sole_names;                //!< a list of device names for the foot pressure sensors
    static vector<string> m_foot_bumper_names;              //!< a list of device names for the foot touch sensors
    static vector<string> m_button_names;                   //!< a list of device names for the buttons
    static vector<string> m_battery_names;                  //!< a list of device names for the battery

    ALMemoryFastAccess* m_al_positions_access;              //!< fast access to almemory position access
    ALMemoryFastAccess* m_al_targets_access;                //!< fast access to almemory target access
    ALMemoryFastAccess* m_al_stiffness_access;              //!< fast access to almemory stiffness access
    ALMemoryFastAccess* m_al_current_access;                //!< fast access to almemory current access
    ALMemoryFastAccess* m_al_temperature_access;            //!< fast access to almemory temperature access
    ALMemoryFastAccess* m_al_accel_access;                  //!< fast access to almemory accelerometer access
    ALMemoryFastAccess* m_al_gyro_access;                   //!< fast access to almemory gyro access
    ALMemoryFastAccess* m_al_footsole_access;               //!< fast access to almemory foot sole access
    ALMemoryFastAccess* m_al_footbumper_access;             //!< fast access to almemory foot bumper access
    ALMemoryFastAccess* m_al_button_access;                 //!< fast access to almemory button access
    ALMemoryFastAccess* m_al_battery_access;                //!< fast access to almemory battery access
    
};

#endif

