/*! @file NAOPlatform.cpp
    @brief Implementation of NAOPlatform

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

#include "NAOPlatform.h"
#include "NAOCamera.h"
#include "NAOSensors.h"
#include "NAOActionators.h"
#include "NAOIO.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "debug.h"
#include "debugverbositynuplatform.h"
#include "Tools/Math/StlVector.h"

/*! @brief Constructor for NAO robotic platform
 */
NAOPlatform::NAOPlatform()
{
    #if DEBUG_NUPLATFORM_VERBOSITY > 1
        debug << "NAOPlatform::NAOPlatform()" << endl;
    #endif
    init();
    m_camera = new NAOCamera();
    m_sensors = new NAOSensors();
    m_actionators = new NAOActionators();
    
    m_battery_state_previous_time = 0;
    m_battery_voiced_time = 0;
}

NAOPlatform::~NAOPlatform()
{
    #if DEBUG_NUPLATFORM_VERBOSITY > 0
        debug << "NAOPlatform::~NAOPlatform()" << endl;
    #endif
}

/*! @brief Displays the battery's charge and the (dis)charge rate on the NAO's ears
 */
void NAOPlatform::displayBatteryState()
{
    float currenttime = Blackboard->Sensors->CurrentTime;
    float period = currenttime - m_battery_state_previous_time;
    
    // get the battery charge and voltage from the sensor data
    float charge, current;
    Blackboard->Sensors->getBatteryCharge(charge);
    Blackboard->Sensors->getBatteryCurrent(current);
    int numleds = 10;
    
    vector<float> leds(numleds, 0);
    // calculate the number of lights to turn on in the ears based on the battery charge
    int numon = (int) (charge*numleds + 0.5);
    for (int i=0; i<numon; i++)
        leds[i] = 1;
    Blackboard->Actions->add(NUActionatorsData::LEarLed, currenttime, leds);
    Blackboard->Actions->add(NUActionatorsData::REarLed, currenttime, leds);

    // do the fancy animated charging/discharging displays
    if (current >= 0 and numon < numleds)
    {   // the battery is charging
        int loops = current/0.5 + 1;
        double timeperloop = period/loops;
        double slope = timeperloop/(numleds - numon);
        for (int l=0; l<loops; l++)
        {
            for (int i=numon; i<numleds; i++)
            {
                vector<float> chargeleds = leds;
                chargeleds[i] = 1;
                Blackboard->Actions->add(NUActionatorsData::LEarLed, currenttime + slope*(i+1 - numon) + l*timeperloop, chargeleds);
                Blackboard->Actions->add(NUActionatorsData::REarLed, currenttime + slope*(i+1 - numon) + l*timeperloop, chargeleds);
            }
        }
    }
    else if (numon > 1)
    {   // the battery is discharging
        int loops = -current/1.0 + 1;
        double timeperloop = period/loops;
        double slope = timeperloop/numon;
        for (int l=0; l<loops; l++)
        {
            for (int i=0; i<numon; i++)
            {
                vector<float> chargeleds = leds;
                chargeleds[(numon-1) - i] = 0;
                Blackboard->Actions->add(NUActionatorsData::LEarLed, currenttime + slope*(i+1) + l*timeperloop, chargeleds);
                Blackboard->Actions->add(NUActionatorsData::REarLed, currenttime + slope*(i+1) + l*timeperloop, chargeleds);
            }
        }
    }
    
    // say low_battery.wav when the battery is really low
    if (charge < 0.02 and current < 0 and currenttime - m_battery_voiced_time > 5000)
    {
        Blackboard->Actions->add(NUActionatorsData::Sound, currenttime, "low_battery.wav");
        m_battery_voiced_time = currenttime;
    }
    
    m_battery_state_previous_time = currenttime;
}

