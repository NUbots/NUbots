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
    static vector<string> m_jointposition_names;
    static vector<string> m_jointtarget_names;
    static vector<string> m_jointstiffness_names;
    static vector<string> m_jointcurrent_names;
    static vector<string> m_jointtemperature_names;
    static vector<string> m_accel_names;
    static vector<string> m_gyro_names;
    static vector<string> m_foot_sole_names;
    static vector<string> m_foot_bumper_names;
    static vector<string> m_button_names;
    static vector<string> m_battery_names;
    static vector<string> m_sensor_names;
    ALMemoryFastAccess* m_al_positions_access;
    ALMemoryFastAccess* m_al_targets_access;
    ALMemoryFastAccess* m_al_stiffness_access;
    ALMemoryFastAccess* m_al_current_access;
    ALMemoryFastAccess* m_al_temperature_access;
    ALMemoryFastAccess* m_al_accel_access;
    ALMemoryFastAccess* m_al_gyro_access;
    ALMemoryFastAccess* m_al_footsole_access;
    ALMemoryFastAccess* m_al_footbumper_access;
    ALMemoryFastAccess* m_al_button_access;
    ALMemoryFastAccess* m_al_battery_access;
    
};

#endif

