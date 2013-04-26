/*! @file DarwinWebotsActionators.cpp
    @brief Implementation of Darwin in Webots actionators class

    @author Jason Kulk, Jed Rietveld
 
  Copyright (c) 2012 Jason Kulk, Jed Rietveld
 
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

#include "DarwinWebotsActionators.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "Tools/Math/StlVector.h"
#include "debug.h"
#include "debugverbositynuactionators.h"

#include "webots/Robot.hpp"
using namespace webots;

#include <string.h>

// init m_servo_names:
static string temp_servo_names[] = {string("HeadPitch"), string("HeadYaw"), \
                                    string("LShoulderRoll"), string("LShoulderPitch"), string("LElbowPitch"), \
                                    string("RShoulderRoll"), string("RShoulderPitch"), string("RElbowPitch"), \
                                    string("LHipRoll"),  string("LHipPitch"), string("LHipYaw"), \
									string("LKneePitch"), string("LAnkleRoll"), string("LAnklePitch"), \
                                    string("RHipRoll"),  string("RHipPitch"), string("RHipYaw"), \
									string("RKneePitch"), string("RAnkleRoll"), string("RAnklePitch")};

vector<string> DarwinWebotsActionators::m_servo_names(temp_servo_names, temp_servo_names + sizeof(temp_servo_names)/sizeof(*temp_servo_names));

// init m_led_names:
static string temp_led_names[] = {string("HeadLed"), string("LEyeLed"), string("REyeLed")};
vector<string> DarwinWebotsActionators::m_led_names(temp_led_names, temp_led_names + sizeof(temp_led_names)/sizeof(*temp_led_names));

// init m_other_names:
static string temp_other_names[] = {/*string("Teleporter"),*/ string("Sound")};
vector<string> DarwinWebotsActionators::m_other_names(temp_other_names, temp_other_names + sizeof(temp_other_names)/sizeof(*temp_other_names));

/*! @brief Constructs a nubot actionator class with a Webots backend
 
    @param platform a pointer to the nuplatform (this is required because webots needs to have nuplatform inherit from the Robot class)
 */ 
DarwinWebotsActionators::DarwinWebotsActionators(DarwinWebotsPlatform* platform) : m_simulation_step(int(platform->getBasicTimeStep()))
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "DarwinWebotsActionators::DarwinWebotsActionators()" << endl;
#endif
    m_platform = platform;
    m_current_time = 0;
    getActionatorsFromWebots(platform);
    enableActionatorsInWebots();
    
    vector<string> names;
    names.insert(names.end(), m_servo_names.begin(), m_servo_names.end());
    names.insert(names.end(), m_led_names.begin(), m_led_names.end());
    names.insert(names.end(), m_other_names.begin(), m_other_names.end());
    m_data->addActionators(names);
    
#if DEBUG_NUACTIONATORS_VERBOSITY > 3
    debug << "DarwinWebotsActionators::DarwinWebotsActionators(). Avaliable Actionators: " << endl;
    m_data->summaryTo(debug);
#endif
}

/*! @brief Get pointers to each of the actionators in the simulated Darwin
 */
void DarwinWebotsActionators::getActionatorsFromWebots(DarwinWebotsPlatform* platform)
{
    // Get the servos
    for (int i=0; i<m_servo_names.size(); i++)
        m_servos.push_back(platform->getServo(m_servo_names[i]));
    // Get the leds
    for (int i=0; i<m_led_names.size(); i++)
        m_leds.push_back(platform->getLED(m_led_names[i]));
    // Get the teleporter
    m_teleporter = platform->getEmitter("super_emitter");
}

/*! @brief enable the actionators in the simulated Darwin
 */
void DarwinWebotsActionators::enableActionatorsInWebots()
{
    // Nothing needs to be done here!
}

DarwinWebotsActionators::~DarwinWebotsActionators()
{
}

void DarwinWebotsActionators::copyToHardwareCommunications()
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 3
    debug << "DarwinWebotsActionators::copyToHardwareCommunications()" << endl;
#endif
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    m_data->summaryTo(debug);
#endif
    
	#if DEBUG_NUACTIONATORS_VERBOSITY > 0
    	debug << "DarwinWebotsActionators::copyToHardwareCommunications():Copying to servos." << endl;
	#endif    
	copyToServos();
	#if DEBUG_NUACTIONATORS_VERBOSITY > 0
    	debug << "DarwinWebotsActionators::copyToHardwareCommunications():Copying to LEDs." << endl;
	#endif
    copyToLeds();
	#if DEBUG_NUACTIONATORS_VERBOSITY > 0
    	debug << "DarwinWebotsActionators::copyToHardwareCommunications():Copying to sound." << endl;
	#endif
    copyToSound();
	#if DEBUG_NUACTIONATORS_VERBOSITY > 0
    	debug << "DarwinWebotsActionators::copyToHardwareCommunications():Copying to teleporter." << endl;
	#endif
    copyToTeleporter();
}

/*! @brief Copies the joint positions and torques to the servos
 */
void DarwinWebotsActionators::copyToServos()
{
    static vector<float> positions;
    static vector<float> gains;
    m_data->getNextServos(positions, gains);   
	for (size_t i=0; i<m_servos.size(); i++)
    {
        JServo* jservo = (JServo*) m_servos[i];
        jservo->setGain(gains[i]);
        jservo->setVelocity(jservo->getMaxVelocity());
        jservo->setPosition(DarwinJointMapping::Instance().joint2rad(i, positions[i]));
    }
}

/*! @brief Copies the led values to the leds
 */
void DarwinWebotsActionators::copyToLeds()
{
    static vector<vector<vector<float> > > ledvalues;
    
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "DarwinWebotsActionators::copyToLeds()" << endl;
#endif
    
	#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    	debug << "DarwinWebotsActionators::copyToLeds():Getting LED values." << endl;
    #endif
	m_data->getNextLeds(ledvalues);
    const unsigned int max_leds = 2;
    const unsigned int total_leds = m_leds.size() < max_leds ? m_leds.size() : max_leds;
    for (size_t i=0; i<total_leds; i++)
    {
		#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    		debug << "DarwinWebotsActionators::copyToLeds():Copying LED #" << i << "." << endl;
		#endif
        unsigned char bgr[3];
        bgr[2] = static_cast<unsigned char> (255*ledvalues[i][0][0]);
        bgr[1] = static_cast<unsigned char> (255*ledvalues[i][0][1]);
        bgr[0] = static_cast<unsigned char> (255*ledvalues[i][0][2]);
        int ledvalue = *reinterpret_cast<int*> (&bgr);
        m_leds[i]->set(ledvalue);
    }
}

/*! @brief Copies the teleportation data to the teleporter (super_emitter)
 */
void DarwinWebotsActionators::copyToTeleporter()
{
    static bool l_isvalid;
    static double l_time;
    static vector<float> l_position(3, 0);
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "DarwinWebotsActionators::copyToTeleporter()" << endl;
#endif 
    /*
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
    }*/
}



