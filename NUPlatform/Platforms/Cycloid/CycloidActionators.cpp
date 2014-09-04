/*! @file CycloidActionators.cpp
    @brief Implementation of cycloid actionators class

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

#include "CycloidActionators.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "../Robotis/Motors.h"

#include <cmath>

#include "debug.h"
#include "debugverbositynuactionators.h"

// init m_servo_names:
static std::string temp_servo_names[] = {std::string("HeadYaw"), \
                                    std::string("LShoulderRoll"), std::string("LShoulderPitch"), std::string("LElbowPitch"), std::string("LElbowYaw"), \
                                    std::string("RShoulderRoll"), std::string("RShoulderPitch"), std::string("RElbowPitch"), std::string("RElbowYaw"), \
                                    std::string("TorsoRoll"), std::string("TorsoPitch"), \
                                    std::string("LHipRoll"),  std::string("LHipPitch"), std::string("LHipYaw"), std::string("LKneePitch"), std::string("LAnkleRoll"), std::string("LAnklePitch"), \
                                    std::string("RHipRoll"),  std::string("RHipPitch"), std::string("RHipYaw"), std::string("RKneePitch"), std::string("RAnkleRoll"), std::string("RAnklePitch")};
std::vector<std::string> CycloidActionators::m_servo_names(temp_servo_names, temp_servo_names + sizeof(temp_servo_names)/sizeof(*temp_servo_names));

/*! @brief Constructs a nubot actionator class with a Bear backend
 */ 
CycloidActionators::CycloidActionators(Motors* motors)
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "CycloidActionators::CycloidActionators()" << std::endl;
#endif
    m_current_time = 0;
    m_motors = motors;
    
    m_data->addActionators(m_servo_names);
    
#if DEBUG_NUACTIONATORS_VERBOSITY > 0
    debug << "CycloidActionators::CycloidActionators(). Avaliable Actionators: " << std::endl;
    m_data->summaryTo(debug);
#endif
}

CycloidActionators::~CycloidActionators()
{
}

void CycloidActionators::copyToHardwareCommunications()
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 3
    debug << "CycloidActionators::copyToHardwareCommunications()" << std::endl;
#endif
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    m_data->summaryTo(debug);
#endif
    copyToServos();
}

void CycloidActionators::copyToServos()
{
    static std::vector<float> positions;
    static std::vector<float> gains;
    
    m_data->getNextServos(positions, gains);
    for (size_t i=0; i<positions.size(); i++)
    {
        // 195.379 converts radians to motor units, and Motors::DefaultPositions are the calibrated zero positions
        float motorposition = Motors::MotorSigns[i]*positions[i]*195.379 + Motors::DefaultPositions[i];                  
        float speed = 1000*fabs(motorposition - JointPositions[i])/(m_data->CurrentTime - m_data->PreviousTime);     
        if (speed > 1023)
            speed = 1023;
        
        if (gains[i] == 0)
            m_motors->torqueOff(Motors::IndexToMotorID[i]);
        else if (gains[i] > 0)
            m_motors->torqueOn(Motors::IndexToMotorID[i]);
        
        m_motors->updateControl(Motors::IndexToMotorID[i], motorposition, speed, -1);
    }
}



