/*! @file DarwinActionators.cpp
    @brief Implementation of darwin actionators class

    @author Jason Kulk
 
  Copyright (c) 2011 Jason Kulk
 
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

#include "DarwinActionators.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include <cmath>

#include "debug.h"
#include "debugverbositynuactionators.h"

/*! @brief Constructs a nubot actionator class with a Darwin backend
            
           The Darwin backend takes aspects from both the NAO and the Robotis backends; the Darwin
           has a secondary board to perform all the communication with hardware, like the NAO. However,
           it also uses Robotis motors, like the Cycloid/Bear.
 
           This backend is also the most recent, and probably should serve as a template for future platforms.
 */ 
DarwinActionators::DarwinActionators()
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "DarwinActionators::DarwinActionators()" << endl;
    #endif
    m_current_time = 0;
    
    /* Make a list of all of the actionators in the Darwin
        We use a standard way of quickly initialising a vector from a normal array that is initialised
        in its declaration
     */
    // start with the joints
    string temp_servo_names[] = {   string("HeadYaw"), string("HeadPitch"), \
                                    string("LShoulderRoll"), string("LShoulderPitch"), string("LElbowPitch"), \
                                    string("RShoulderRoll"), string("RShoulderPitch"), string("RElbowPitch"), \
                                    string("LHipRoll"),  string("LHipPitch"), string("LHipYaw"), string("LKneePitch"), string("LAnkleRoll"), string("LAnklePitch"), \
                                    string("RHipRoll"),  string("RHipPitch"), string("RHipYaw"), string("RKneePitch"), string("RAnkleRoll"), string("RAnklePitch")};
    vector<string> m_servo_names(temp_servo_names, temp_servo_names + sizeof(temp_servo_names)/sizeof(*temp_servo_names));
    
    m_data->addActionators(m_servo_names);
    
    #if DEBUG_NUACTIONATORS_VERBOSITY > 0
        debug << "DarwinActionators::DarwinActionators(). Avaliable Actionators: " << endl;
        m_data->summaryTo(debug);
    #endif
}

DarwinActionators::~DarwinActionators()
{
}

void DarwinActionators::copyToHardwareCommunications()
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 3
        debug << "DarwinActionators::copyToHardwareCommunications()" << endl;
    #endif
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        m_data->summaryTo(debug);
    #endif
    copyToServos();
    copyToLeds();
}

void DarwinActionators::copyToServos()
{
    
}

void DarwinActionators::copyToLeds()
{
    
}



