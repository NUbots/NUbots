/*! @file NAOPlatform.h
    @brief Declaration of NAO platform class.

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

#ifndef NAOPLATFORM_H
#define NAOPLATFORM_H

#include "NUPlatform/NUPlatform.h"

class NAOPlatform : public NUPlatform
{
public:
    NAOPlatform();
    ~NAOPlatform();
    
    // NAO specific functions
    bool displayBatteryState();
    bool verifySensors();
    void add(const LedIndices& led, double time, const std::vector<float>& value);
    void toggle(const LedIndices& led, double time, const std::vector<float>& value);
    
private:
    // members for the battery display
    float m_battery_state_previous_time;
    float m_battery_voiced_time;
    
    // members for the sensor verification
    int m_bad_foot_sensor_count;
    float m_previous_lfoot_force;
    float m_previous_rfoot_force;
    int m_bad_ultrasonic_count;
    int m_heat_count;
    
    // members for the eye leds
    std::vector<std::vector<float> > m_eye_indices; 
    std::vector<std::vector<float> > m_leye;
    std::vector<std::vector<float> > m_reye;
};

#endif

