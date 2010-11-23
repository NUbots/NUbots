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
#include "NUPlatform/NUActionators/NUSounds.h"

#include "debug.h"
#include "debugverbositynusystem.h"

#include <stdio.h>
#include <stdlib.h>
using namespace std;

NAOSystem::NAOSystem()
{
#if DEBUG_NUSYSTEM_VERBOSITY > 4
    debug << "NAOSystem::NAOSystem()" << endl;
#endif
    m_battery_state_previous_time = 0;
    
    m_team_received_leds = vector<vector<float> >(1, vector<float>(3,0.0f));
    m_team_sent_leds = vector<vector<float> >(1, vector<float>(3,0.0f));
    m_game_received_leds = vector<vector<float> >(1, vector<float>(3,0.0f));
    m_frame_drop_leds = vector<vector<float> >(1, vector<float>(3,0.0f));
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
    m_current_time = data->CurrentTime;
    m_period = m_current_time - m_battery_state_previous_time;
    // get the battery charge from the sensor data
    vector<float> battery;
    data->getBatteryValues(battery);
    float charge = 1.0;
    if (battery.size() > 0)
        charge = battery[0];
    
    vector<float> ledoff(3,0);
    vector<float> ledon(3,1);
    // calculate the number of lights to turn on in the ears based on the battery charge
    int numleds = actions->getNumberOfLeds(NUActionatorsData::LeftEarLeds);
    m_ear_leds = vector<vector<float> >(numleds, ledoff);
    int numon = (int) (charge*numleds + 0.5);
    for (int i=0; i<numon; i++)
        m_ear_leds[i] = ledon;
    actions->addLeds(NUActionatorsData::LeftEarLeds, m_current_time, m_ear_leds);
    actions->addLeds(NUActionatorsData::RightEarLeds, m_current_time, m_ear_leds);
    
    float current = 0;
    if (battery.size() > 1)
        current = battery[1];
    if (current >= 0 and numon < numleds)
    {   // the battery is charging
        int loops = current/0.5 + 1;
        double timeperloop = m_period/loops;
        double slope = timeperloop/(numleds - numon);
        for (int l=0; l<loops; l++)
        {
            for (int i=numon; i<numleds; i++)
            {
                vector<vector<float> > chargeleds = m_ear_leds;
                chargeleds[i] = ledon;
                actions->addLeds(NUActionatorsData::LeftEarLeds, m_current_time + slope*(i - numon) + l*timeperloop, chargeleds);
                actions->addLeds(NUActionatorsData::RightEarLeds, m_current_time + slope*(i - numon) + l*timeperloop, chargeleds);
            }
        }
    }
    else if (numon > 1)
    {   // the battery is discharging
        int loops = -current/1.0 + 1;
        double timeperloop = m_period/loops;
        double slope = timeperloop/numon;
        for (int l=0; l<loops; l++)
        {
            for (int i=0; i<numon; i++)
            {
                vector<vector<float> > chargeleds = m_ear_leds;
                chargeleds[(numon-1) - i] = vector<float> (3, 0.0);
                actions->addLeds(NUActionatorsData::LeftEarLeds, m_current_time + slope*i + l*timeperloop, chargeleds);
                actions->addLeds(NUActionatorsData::RightEarLeds, m_current_time + slope*i + l*timeperloop, chargeleds);
            }
        }
    }
    if (charge < 0.02 and current < 0)
        voiceLowBattery(actions);
    
    m_battery_state_previous_time = m_current_time;
}

/*! @brief Voices that the battery is low
 */
void NAOSystem::voiceLowBattery(NUActionatorsData* actions)
{
    static int runcount = 0;
    if (runcount%8 == 0)
        actions->addSound(m_current_time, NUSounds::LOW_BATTERY);
    runcount++;
}

/*! @brief Displays that a team packet has been successfully received
 
    The first quarter of the left eye will toggle each time a packet is received.
 
    @param actions a pointer to the shared actionator object
 */
void NAOSystem::displayTeamPacketReceived(NUActionatorsData* actions)
{
    if (actions == NULL)
        return;
    
    int numleds = actions->getNumberOfLeds(NUActionatorsData::LeftEyeLeds)/4;
    
    vector<int> indices;
    indices.reserve(numleds);
    for (int i=1; i<numleds+1; i++)         // 1,2
        indices.push_back(i);
    
    if (m_team_received_leds[0][1] == 1)
        m_team_received_leds[0][1] = 0;
    else
        m_team_received_leds[0][1] = 1;
    actions->addLeds(NUActionatorsData::LeftEyeLeds, indices, 0, m_team_received_leds);
}

/*! @brief Displays that a team packet has been successfully sent
 @param actions a pointer to the shared actionator object
 */
void NAOSystem::displayTeamPacketSent(NUActionatorsData* actions)
{
    if (actions == NULL)
        return;
    
    int numleds = actions->getNumberOfLeds(NUActionatorsData::LeftEyeLeds)/4;
    
    vector<int> indices;
    indices.reserve(numleds);
    for (int i=numleds*2+1; i<(numleds*3 +1); i++)    // 5,6
        indices.push_back(i);
    
    if (m_team_sent_leds[0][1] == 1)
        m_team_sent_leds[0][1] = 0;
    else
        m_team_sent_leds[0][1] = 1;
    actions->addLeds(NUActionatorsData::LeftEyeLeds, indices, 0, m_team_sent_leds);
}

/*! @brief Displays that a game packet has been successfully received
    @param actions a pointer to the shared actionator object
 */
void NAOSystem::displayGamePacketReceived(NUActionatorsData* actions)
{
    if (actions == NULL)
        return;
    
    int numleds = actions->getNumberOfLeds(NUActionatorsData::LeftEyeLeds)/4;
    
    vector<int> indices;
    indices.reserve(numleds);
    for (int i=(numleds +1); i<(numleds*2 +1); i++)     // 3,4
        indices.push_back(i);
    
    if (m_game_received_leds[0][0] == 1)
        m_game_received_leds[0][0] = 0;
    else
        m_game_received_leds[0][0] = 1;
    actions->addLeds(NUActionatorsData::LeftEyeLeds, indices, actions->CurrentTime, m_game_received_leds);
}

/*! @brief Displays that a packet has been successfully received
 @param actions a pointer to the shared actionator object
 */
void NAOSystem::displayOtherPacketReceived(NUActionatorsData* actions)
{
    // by default there is no way to display such information!
}

/*! @brief Display some sign that a vision frame has been dropped
    @param actions a pointer to the shared actionator object
 */
void NAOSystem::displayVisionFrameDrop(NUActionatorsData* actions)
{
    if (actions == NULL)
        return;
    
    static vector<int> indices(2,0);
    indices[1] = 7;
    m_frame_drop_leds[0][0] = 1;
    actions->addLeds(NUActionatorsData::LeftEyeLeds, indices, actions->CurrentTime, m_frame_drop_leds);
    m_frame_drop_leds[0][0] = 0;
    actions->addLeds(NUActionatorsData::LeftEyeLeds, indices, actions->CurrentTime + 200, m_frame_drop_leds);
}

void NAOSystem::restart()
{
    system("/etc/init.d/naoqi restart");        // this doesn't work
}



