/*! @file NAOSystem.h
    @brief Declaration of NAO system class

    @author Jason Kulk
 
    @class NAOSystem
    @brief NAO system class
 
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

#ifndef NAOSYSTEM_H
#define NAOSYSTEM_H

#include "NUPlatform/NUSystem.h"

#include <vector>

class NAOSystem : public NUSystem
{
public:
    NAOSystem();
    ~NAOSystem();
    
    // battery functions
    void displayBatteryState(NUSensorsData* data, NUActionatorsData* actions);
    void voiceLowBattery(NUActionatorsData* actions);
    // watchdog functions
    void displayVisionFrameDrop(NUActionatorsData* actions);
    void voiceFrameDrop(NUActionatorsData* actions);
    void displayGamePacketReceived(NUActionatorsData* actions);
    void displayTeamPacketReceived(NUActionatorsData* actions);
    
private:
    double m_current_time;
    double m_battery_state_previous_time;                    //!< the previous time displayBatteryState was called
    double m_period;                                         //!< period between ear led updates
    std::vector<std::vector<float> > m_ear_leds;             //!< the current battery level ear led values
};

#endif

