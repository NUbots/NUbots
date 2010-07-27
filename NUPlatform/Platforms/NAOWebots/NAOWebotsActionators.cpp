/*! @file NAOWebotsActionators.cpp
    @brief Implementation of NAO in Webots actionators class

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

#include "NAOWebotsActionators.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "debug.h"
#include "debugverbositynuactionators.h"

#include "webots/Robot.hpp"
using namespace webots;

#include <string.h>


// init m_actionator_names:
static string temp_servo_control_names[] = {string("JointPositions"), string("JointTorques")};
vector<string> NAOWebotsActionators::m_servo_control_names(temp_servo_control_names, temp_servo_control_names + sizeof(temp_servo_control_names)/sizeof(*temp_servo_control_names));

// init m_servo_names:
static string temp_servo_names[] = {string("HeadPitch"), string("HeadYaw"), \
                                    string("LShoulderRoll"), string("LShoulderPitch"), string("LElbowRoll"), string("LElbowYaw"), \
                                    string("RShoulderRoll"), string("RShoulderPitch"), string("RElbowRoll"), string("RElbowYaw"), \
                                    string("LHipRoll"),  string("LHipPitch"), string("LHipYawPitch"), string("LKneePitch"), string("LAnkleRoll"), string("LAnklePitch"), \
                                    string("RHipRoll"),  string("RHipPitch"), string("RHipYawPitch"), string("RKneePitch"), string("RAnkleRoll"), string("RAnklePitch")};
vector<string> NAOWebotsActionators::m_servo_names(temp_servo_names, temp_servo_names + sizeof(temp_servo_names)/sizeof(*temp_servo_names));

// init m_led_names:
static string temp_led_names[] = {string("Ears/Led/Left"), string("Ears/Led/Right"), string("Face/Led/Left"), string("Face/Led/Right"), \
                                  string("ChestBoard/Led"), \
                                  string("LFoot/Led"), string("RFoot/Led")};
vector<string> NAOWebotsActionators::m_led_names(temp_led_names, temp_led_names + sizeof(temp_led_names)/sizeof(*temp_led_names));
// init m_other_names:
static string temp_other_names[] = {string("Teleporter"), string("Sound")};
vector<string> NAOWebotsActionators::m_other_names(temp_other_names, temp_other_names + sizeof(temp_other_names)/sizeof(*temp_other_names));

/*! @brief Constructs a nubot actionator class with a Webots backend
 
    @param platform a pointer to the nuplatform (this is required because webots needs to have nuplatform inherit from the Robot class)
 */ 
NAOWebotsActionators::NAOWebotsActionators(NAOWebotsPlatform* platform) : m_simulation_step(int(platform->getBasicTimeStep()))
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NAOWebotsActionators::NAOWebotsActionators()" << endl;
#endif
    m_platform = platform;
    m_current_time = 0;
    getActionatorsFromWebots(platform);
    enableActionatorsInWebots();
    
    m_data->setAvailableJointControlMethods(m_servo_control_names);
    m_data->setAvailableJoints(m_servo_names);
    m_data->setAvailableLeds(m_led_names);
    m_data->setAvailableOtherActionators(m_other_names);
    
#if DEBUG_NUACTIONATORS_VERBOSITY > 3
    debug << "NAOWebotsActionators::NAOWebotsActionators(). Avaliable Actionators: " << endl;
    m_data->summaryTo(debug);
#endif
}

/*! @brief Get pointers to each of the actionators in the simulated NAO
 */
void NAOWebotsActionators::getActionatorsFromWebots(NAOWebotsPlatform* platform)
{
    // Get the servos
    for (int i=0; i<m_servo_names.size(); i++)
        m_servos.push_back(platform->getServo(m_servo_names[i]));
    // Get the leds
    for (int i=0; i<m_led_names.size(); i++)
        m_leds.push_back(platform->getLED(m_led_names[i]));
    //! @todo TODO: get the sound from webots
    // Get the teleporter
    m_teleporter = platform->getEmitter("super_emitter");
}

/*! @brief enable the actionators in the simulated NAO
 */
void NAOWebotsActionators::enableActionatorsInWebots()
{
    // Nothing needs to be done here!
}

NAOWebotsActionators::~NAOWebotsActionators()
{
}

void NAOWebotsActionators::copyToHardwareCommunications()
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 3
    debug << "NAOWebotsActionators::copyToHardwareCommunications()" << endl;
#endif
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    m_data->summaryTo(debug);
#endif
    
    copyToServos();
    copyToLeds();
    copyToSound();
    copyToTeleporter();
}

/*! @brief Copies the joint positions and torques to the servos
 */
void NAOWebotsActionators::copyToServos()
{
    static vector<bool> isvalid;            
    static vector<double> times;
    static vector<float> positions;
    static vector<float> velocities;
    static vector<float> torques;
    static vector<float> gains;
    
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NAOWebotsActionators::copyToServos()" << endl;
#endif
    
    // Positions
    if (m_data->getNextJointPositions(isvalid, times, positions, velocities, gains))
    {
        if (m_servos.size() == isvalid.size())                          // only process the input if it has the right length
        {
            for (int i=0; i<m_servos.size(); i++)
            {
                if (isvalid[i] == true)     // I need to put in a bit of a hack here, because Webots actually allows for left and right hip yaw 
                {
                    if (i == NUActionatorsData::RHipYawPitch)
                        positions[i] = positions[NUActionatorsData::LHipYawPitch];
                    
                    JServo* jservo = (JServo*) m_servos[i];
                    if (times[i] >= m_current_time)
                    {
                        float c = jservo->getPosition();           // i think I am allowed to do this right? I ought to be I am only emulating (time, position) available on other platforms!
                        float dt = times[i] - m_current_time;
                        float v = 1000*(positions[i] - c)/dt;
                        jservo->setGain(gains[i]);
                        jservo->setVelocity(v);
                        jservo->setPosition(positions[i]);
                    }
                    else
                    {   // the command has already past, we should get there as fast as possible
                        jservo->setGain(gains[i]);
                        jservo->setVelocity(jservo->getMaxVelocity());
                        jservo->setPosition(positions[i]);
                    }
                }
            }
        }
        else
            debug << "NAOWebotsActionators::copyToServos(). The input does not have the correct length, all data will be ignored!" << endl;

    }
    // Torques
    if (m_data->getNextJointTorques(isvalid, times, torques, gains))
    {
        if (m_servos.size() == isvalid.size())
        {
            for (int i=0; i<m_servos.size(); i++)
            {
                if (isvalid[i] == true)
                    m_servos[i]->setForce(torques[i]);          // I think this is broken in webots!
            }
        }
        else
            debug << "NAOWebotsActionators::copyToServos(). The input does not have the correct length, all data will be ignored!" << endl;
    }
}

/*! @brief Copies the led values to the leds
 */
void NAOWebotsActionators::copyToLeds()
{
    static vector<bool> isvalid;
    static vector<double> times;
    static vector<float> redvalues;
    static vector<float> greenvalues;
    static vector<float> bluevalues;
    
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NAOWebotsActionators::copyToLeds()" << endl;
#endif
    
    if (m_data->getNextLeds(isvalid, times, redvalues, greenvalues, bluevalues))
    {
        if (m_leds.size() == isvalid.size())
        {
            for (int i=0; i<m_leds.size(); i++)
            {
                if (isvalid[i] == true)
                {
                    unsigned char bgr[3];
                    bgr[2] = static_cast<unsigned char> (255*redvalues[i]);
                    bgr[1] = static_cast<unsigned char> (255*greenvalues[i]);
                    bgr[0] = static_cast<unsigned char> (255*bluevalues[i]);
                    int ledvalue = *reinterpret_cast<int*> (&bgr);
                    m_leds[i]->set(ledvalue);
                }
            }
        }
        else
            debug << "NAOWebotsActionators::copyToLeds(). The input does not have the correct length, all data will be ignored!" << endl;
    }
}

/*! @brief Copies the teleportation data to the teleporter (super_emitter)
 */
void NAOWebotsActionators::copyToTeleporter()
{
    static bool l_isvalid;
    static double l_time;
    static vector<float> l_position(3, 0);
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NAOWebotsActionators::copyToTeleporter()" << endl;
#endif 
    if (m_data->getNextTeleportation(l_isvalid, l_time, l_position))
    {
        if (l_isvalid == true)// && (l_time - m_current_time) < m_simulation_step)
        {
            static char buf[256];
            static char teamred[] = "RED";
            static char teamblue[] = "BLUE";
            static int id;             // webots id = id - 1
            // get the player id
            id = m_platform->getRobotNumber();
            id--;
            
            // get the player's colour
            int teamid = m_platform->getTeamNumber();
            char* team;
            if (teamid == 0)
                team = teamblue;
            else
                team = teamred;
            
            // convert from our standard coordinates to webots teleporter coords (I can do the conversion in-line cause it is simple)
            // x is toward yellow goal, y is up and z is right
            sprintf(buf, "move robot %s %d %f %f %f %f", team, id, l_position[0]/100.0, 35.0/100.0, -l_position[1]/100.0, l_position[2] + 3.141/2.0);
            m_teleporter->send(buf, strlen(buf) + 1);
        }
    }
}



