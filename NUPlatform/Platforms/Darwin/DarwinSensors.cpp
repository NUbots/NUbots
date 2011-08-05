/*! @file DarwinSensors.cpp
    @brief Implementation of Darwin sensor class

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

#include "DarwinSensors.h"
#include "../Robotis/Motors.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"

#include "Tools/Math/General.h"

#include "debug.h"
#include "debugverbositynusensors.h"

#include <limits>
using namespace std;

/*! @brief Constructs a nubot sensor class with Bear backend
 */
DarwinSensors::DarwinSensors()
{
    #if DEBUG_NUSENSORS_VERBOSITY > 0
        debug << "DarwinSensors::DarwinSensors()" << endl;
    #endif
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

    m_data->addSensors(m_servo_names);

    m_previous_positions = vector<float>(m_servo_names.size(), 0);
    m_previous_velocities = vector<float>(m_servo_names.size(), 0);
}

/*! @brief Destructor for DarwinSensors
 */
DarwinSensors::~DarwinSensors()
{
    #if DEBUG_NUSENSORS_VERBOSITY > 0
        debug << "DarwinSensors::~DarwinSensors()" << endl;
    #endif
}

/*! @brief Copys the sensors data from the hardware communication module to the NUSensorsData container
 */
void DarwinSensors::copyFromHardwareCommunications()
{
    copyFromJoints();
    copyFromAccelerometerAndGyro();
    copyFromFeet();
    copyFromButtons();
    copyFromBattery();
}

/*! @brief Copys the joint sensor data 
 */
void DarwinSensors::copyFromJoints()
{
}

void DarwinSensors::copyFromAccelerometerAndGyro()
{
}

void DarwinSensors::copyFromFeet()
{
}

void DarwinSensors::copyFromButtons()
{
}

void DarwinSensors::copyFromBattery()
{
}

