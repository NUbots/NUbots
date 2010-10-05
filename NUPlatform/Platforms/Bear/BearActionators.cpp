/*! @file BearActionators.cpp
    @brief Implementation of Bear actionators class

    @author Jason Kulk
 
  Copyright (c) 2010 Jason Kulk
 
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

#include "BearActionators.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Serial/Motors.h"

#include <cmath>

#include "debug.h"
#include "debugverbositynuactionators.h"

// init m_actionator_names:
static string temp_servo_control_names[] = {string("JointPositions"), string("JointTorques")};
vector<string> BearActionators::m_servo_control_names(temp_servo_control_names, temp_servo_control_names + sizeof(temp_servo_control_names)/sizeof(*temp_servo_control_names));

// init m_servo_names:
static string temp_servo_names[] = {string("HeadPitch"), string("HeadYaw"), \
                                    string("TorsoPitch"), \
                                    string("LShoulderRoll"), string("LShoulderPitch"), string("LElbowRoll"), \
                                    string("RShoulderRoll"), string("RShoulderPitch"), string("RElbowRoll"), \
                                    string("TorsoRoll"), string("TorsoYaw"), \
                                    string("LHipRoll"),  string("LHipPitch"), string("LKneePitch"), string("LAnkleRoll"), string("LAnklePitch"), \
                                    string("RHipRoll"),  string("RHipPitch"), string("RKneePitch"), string("RAnkleRoll"), string("RAnklePitch")};
vector<string> BearActionators::m_servo_names(temp_servo_names, temp_servo_names + sizeof(temp_servo_names)/sizeof(*temp_servo_names));

/*! @brief Constructs a nubot actionator class with a Bear backend
 */ 
BearActionators::BearActionators(Motors* motors)
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "BearActionators::BearActionators()" << endl;
#endif
    m_current_time = 0;
    m_motors = motors;
    
    m_data->setAvailableJointControlMethods(m_servo_control_names);
    m_data->setAvailableJoints(m_servo_names);
    
#if DEBUG_NUACTIONATORS_VERBOSITY > 0
    debug << "BearActionators::BearActionators(). Avaliable Actionators: " << endl;
    m_data->summaryTo(debug);
#endif
}

BearActionators::~BearActionators()
{
}

void BearActionators::copyToHardwareCommunications()
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 3
    debug << "BearActionators::copyToHardwareCommunications()" << endl;
#endif
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    m_data->summaryTo(debug);
#endif
    vector<bool> isvalid;
    vector<double> time;
    vector<float> position;
    vector<float> velocity;
    vector<float> gain;
    
    if (m_data->getNextJointPositions(isvalid, time, position, velocity, gain))
    {
        for (int i=0; i<position.size(); i++)
        {
            if (isvalid[i])
            {
                // 195.379 converts radians to motor units, and Motors::DefaultPositions are the calibrated zero positions
                float motorposition = Motors::MotorSigns[i]*position[i]*195.379 + Motors::DefaultPositions[i];                  
                float speed = 1000*fabs(motorposition - JointPositions[i])/(time[i] - m_current_time);     
                
                m_motors->updateControl(Motors::IndexToMotorID[i], motorposition, speed, -1);
            }
        }
    }
    
}



