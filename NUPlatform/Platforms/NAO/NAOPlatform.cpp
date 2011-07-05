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
    
    m_bad_foot_sensor_count = 0;
    m_bad_ultrasonic_count = 0;
    m_heat_count = 0;
    
    m_eye_indices = vector<vector<float> >(4, vector<float>(2,0));
    m_eye_indices[0][0] = 0;
    m_eye_indices[0][1] = 7;
    m_eye_indices[1][0] = 1;
    m_eye_indices[1][1] = 2;
    m_eye_indices[2][0] = 3;
    m_eye_indices[2][1] = 4;
    m_eye_indices[3][0] = 5;
    m_eye_indices[3][1] = 6;
    
    m_leye = vector<vector<float> >(8, vector<float>(3,0));
    m_reye = vector<vector<float> >(8, vector<float>(3,0));
}

NAOPlatform::~NAOPlatform()
{
    #if DEBUG_NUPLATFORM_VERBOSITY > 0
        debug << "NAOPlatform::~NAOPlatform()" << endl;
    #endif
}

/*! @brief Displays the battery's charge and the (dis)charge rate on the NAO's ears
 */
bool NAOPlatform::displayBatteryState()
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
    bool ok = true;
    if (charge < 0.02 and current < 0 and currenttime - m_battery_voiced_time > 5000)
    {
        Blackboard->Actions->add(NUActionatorsData::Sound, currenttime, "low_battery.wav");
        m_battery_voiced_time = currenttime;
        ok = false;
    }
    
    m_battery_state_previous_time = currenttime;
    return ok;
}

/*! @brief Verifys that the sensors are operating properly. In particular, the foot sensors and ultrasonics are checked. */
bool NAOPlatform::verifySensors()
{
    // check foot sensors are changing
    float lf,rf;
    bool f_valid = Blackboard->Sensors->getForce(NUSensorsData::LFoot, lf);
    f_valid &= Blackboard->Sensors->getForce(NUSensorsData::RFoot, rf);
    
    if (f_valid)
    {
        if (lf == m_previous_lfoot_force or rf == m_previous_rfoot_force)
            m_bad_foot_sensor_count++;
        else
            m_bad_foot_sensor_count = 0;
        
        m_previous_lfoot_force = lf;
        m_previous_rfoot_force = rf;
    }
    
    bool ok = true;
    if (m_bad_foot_sensor_count >= 5)
    {
        Blackboard->Actions->add(NUActionatorsData::Sound, Blackboard->Actions->CurrentTime, "error_foot_sensors.wav");
        m_bad_foot_sensor_count = 0;
        ok = false;
    }
    
    // check ultrasonics for invalid readings
    vector<float> lu,ru;
    bool u_valid = Blackboard->Sensors->getDistance(NUSensorsData::LDistance, lu);
    u_valid = Blackboard->Sensors->getDistance(NUSensorsData::RDistance, ru);
    
    if (u_valid and not lu.empty() and not ru.empty())
    {
        if (lu[0] <= 2 or ru[0] <= 2)
            m_bad_ultrasonic_count++;
        else
            m_bad_ultrasonic_count = 0;
    }
    
    if (m_bad_ultrasonic_count >= 5)
    {
        Blackboard->Actions->add(NUActionatorsData::Sound, Blackboard->Actions->CurrentTime, "error_ultrasonic_sensors.wav");
        m_bad_ultrasonic_count = 0;
        ok = false;
    }
    return ok;
}

/*! @brief Sets one of the eight eye quarters to the given (time,value)
 */
void NAOPlatform::add(const LedIndices& led, double time, const vector<float>& value)
{
    vector<float> indices = m_eye_indices[led%4];
    if (led < 4)
    {
        m_leye[indices[0]] = value;
        m_leye[indices[1]] = value;
        Blackboard->Actions->add(NUActionatorsData::LEyeLed, time, m_leye);
    }
    else
    {
        m_reye[indices[0]] = value;
        m_reye[indices[1]] = value;
        Blackboard->Actions->add(NUActionatorsData::REyeLed, time, m_reye);
    }
}

/*! @brief Toggles one of the eight eye quarters to the given (time,value)
 */
void NAOPlatform::toggle(const LedIndices& led, double time, const vector<float>& value)
{
    vector<float> indices = m_eye_indices[led%4];
    if (led < 4)
    {
        if (m_leye[indices[0]] == value or m_leye[indices[1]] == value)
        {
            m_leye[indices[0]] = vector<float>(3,0);
            m_leye[indices[1]] = vector<float>(3,0);
        }
        else 
        {
            m_leye[indices[0]] = value;
            m_leye[indices[1]] = value;
        }
        Blackboard->Actions->add(NUActionatorsData::LEyeLed, time, m_leye);
    }
    else
    {
        if (m_reye[indices[0]] == value or m_reye[indices[1]] == value)
        {
            m_reye[indices[0]] = vector<float>(3,0);
            m_reye[indices[1]] = vector<float>(3,0);
        }
        else 
        {
            m_reye[indices[0]] = value;
            m_reye[indices[1]] = value;
        }
        Blackboard->Actions->add(NUActionatorsData::REyeLed, time, m_leye);
    }
}
