/*! @file NAOSystem.cpp
    @brief Implementation of NAOsystem class

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

#include "NAOSystem.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"

#include "debug.h"
#include "debugverbositynusystem.h"

using namespace std;

NAOSystem::NAOSystem()
{
#if DEBUG_NUSYSTEM_VERBOSITY > 4
    debug << "NAOSystem::NAOSystem()" << endl;
#endif
    m_battery_state_previous_time = 0;
}

NAOSystem::~NAOSystem()
{
}

/*! @brief Display the current charge and current drain of the battery
    @param data a pointer to the shared sensor data object (it contains the battery values)
    @param actions a pointer to the shared actionator object
 */
void NAOSystem::displayBatteryState(NUSensorsData* data, NUActionatorsData* actions)
{
    if (data == NULL || actions == NULL)
        return;
    double currenttime = data->CurrentTime;
    double period = currenttime - m_battery_state_previous_time;
    // get the battery charge from the sensor data
    vector<float> battery;
    data->getBatteryValues(battery);
    float charge = 1.0;
    if (battery.size() > 0)
        charge = battery[0];
    
    // calculate the number of lights to turn on in the ears based on the battery charge
    int numleds = actions->getNumberOfLeds(NUActionatorsData::LeftEarLeds);
    vector<vector<float> > leds(numleds, vector<float>(3,0));
    int numon = (int) (charge*numleds + 0.5);
    for (int i=0; i<numon; i++)
        leds[i] = vector<float> (3, 1.0);
    actions->addLeds(NUActionatorsData::LeftEarLeds, currenttime, leds);
    actions->addLeds(NUActionatorsData::RightEarLeds, currenttime, leds);
    
    float current = 0;
    if (battery.size() > 1)
        current = battery[1];
    if (current >= 0 && numon < numleds)
    {   // the battery is charging
        double slope = period/(numleds - numon);
        for (int i=numon; i<numleds; i++)
        {
            vector<vector<float> > chargeleds = leds;
            chargeleds[i] = vector<float> (3, 1.0);
            actions->addLeds(NUActionatorsData::LeftEarLeds, currenttime + slope*(i - numon), chargeleds);
            actions->addLeds(NUActionatorsData::RightEarLeds, currenttime + slope*(i - numon), chargeleds);
        }
    }
    else
    {   // the battery is discharging
        double slope = period/numon;
        for (int i=0; i<numon; i++)
        {
            vector<vector<float> > chargeleds = leds;
            chargeleds[(numon-1) - i] = vector<float> (3, 0.0);
            actions->addLeds(NUActionatorsData::LeftEarLeds, currenttime + slope*i, chargeleds);
            actions->addLeds(NUActionatorsData::RightEarLeds, currenttime + slope*i, chargeleds);
        }
    }
    
    m_battery_state_previous_time = data->CurrentTime;
}

/*! @brief Display some sign that the code is running OK
    @param actions a pointer to the shared actionator object
 */
void NAOSystem::displayRunning(NUActionatorsData* actions)
{
    // by default there is no way to display such information!
}

/*! @brief Display some sign that a vision frame has been dropped
    @param actions a pointer to the shared actionator object
 */
void NAOSystem::displayVisionFrameDrop(NUActionatorsData* actions)
{
    // by default there is no way to display such information!
}





