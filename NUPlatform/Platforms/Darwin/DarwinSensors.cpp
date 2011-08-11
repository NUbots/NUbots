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
#include "DarwinSensorNames.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"

//Darwin Motors:
#include <MX28.h>

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
                                    string("LHipRoll"),  string("LHipPitch"), string("LHipYaw"),  \
									string("LKneePitch"), string("LAnkleRoll"), string("LAnklePitch"), \
                                    string("RHipRoll"),  string("RHipPitch"), string("RHipYaw"), \
									string("RKneePitch"), string("RAnkleRoll"), string("RAnklePitch")};

	int temp_servo_IDs[] = 		{	JointID::ID_HEAD_PAN, JointID::ID_HEAD_TILT, \
									JointID::ID_L_SHOULDER_ROLL, JointID::ID_L_SHOULDER_PITCH, JointID::ID_L_ELBOW, \
									JointID::ID_R_SHOULDER_ROLL, JointID::ID_R_SHOULDER_PITCH, JointID::ID_R_ELBOW, \
									JointID::ID_L_HIP_ROLL, JointID::ID_L_HIP_PITCH, JointID::ID_L_HIP_YAW, \
									JointID::ID_L_KNEE, JointID::ID_L_ANKLE_ROLL, JointID::ID_L_ANKLE_PITCH, \
									JointID::ID_R_HIP_ROLL, JointID::ID_R_HIP_PITCH, JointID::ID_R_HIP_YAW, \
									JointID::ID_R_KNEE, JointID::ID_R_ANKLE_ROLL, JointID::ID_R_ANKLE_PITCH};

    m_servo_names = vector<string>(temp_servo_names, temp_servo_names + sizeof(temp_servo_names)/sizeof(*temp_servo_names));
	m_servo_IDs = vector<int>(temp_servo_IDs, temp_servo_IDs + sizeof(temp_servo_IDs)/sizeof(*temp_servo_IDs));
    m_data->addSensors(m_servo_names);

	m_joint_ids = m_data->mapIdToIds(NUSensorsData::All);
    m_previous_positions = vector<float>(m_servo_names.size(), 0);
    m_previous_velocities = vector<float>(m_servo_names.size(), 0);

	//Code to Connect to Darwin SubController [Taken from Read/Write Tutorial]: 
	linux_cm730 = new Robot::LinuxCM730("/dev/ttyUSB0");
	cm730 = new Robot::CM730(linux_cm730);
	if(cm730->Connect() == false)
	{
		printf("Fail to connect CM-730!\n");
		return;
	}
}

/*! @brief Destructor for DarwinSensors
 */
DarwinSensors::~DarwinSensors()
{
    #if DEBUG_NUSENSORS_VERBOSITY > 0
        debug << "DarwinSensors::~DarwinSensors()" << endl;
    #endif
	delete cm730;
	delete linux_cm730;
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
	
	static const float NaN = numeric_limits<float>::quiet_NaN();
	vector<float> joint(NUSensorsData::NumJointSensorIndices, NaN);
    float delta_t = (m_current_time - m_previous_time)/1000;
	int data;
	for (size_t i=0; i<m_servo_IDs.size(); i++)
    {
		
		cm730->ReadWord(int(m_servo_IDs[i]),int(Robot::MX28::P_PRESENT_POSITION_L), &(data), 0); 	//<! Read Position
		joint[NUSensorsData::PositionId] = Robot::MX28::Value2Angle(data);
		cm730->ReadWord(int(m_servo_IDs[i]),int(Robot::MX28::P_MOVING_SPEED_L), &(data), 0); 		//<! Read Velocity
		joint[NUSensorsData::VelocityId] = data;
		cm730->ReadWord(int(m_servo_IDs[i]),int(Robot::MX28::P_GOAL_POSITION_L), &(data), 0); 			//<! Read Goal Position (target)
		joint[NUSensorsData::TargetId] = data;
		cm730->ReadByte(int(m_servo_IDs[i]),int(Robot::MX28::P_PRESENT_TEMPERATURE), &(data), 0); //<! Read Temperature
		joint[NUSensorsData::TemperatureId] = data;
		cm730->ReadByte(int(m_servo_IDs[i]),int(Robot::MX28::P_TORQUE_ENABLE), &(data), 0); 		//<! Read Stiffness
		joint[NUSensorsData::StiffnessId] = data;
		cm730->ReadWord(int(m_servo_IDs[i]),int(Robot::MX28::P_PRESENT_LOAD_L), &(data), 0); 			//<! Read Goal Position (target)
		joint[NUSensorsData::TorqueId] = data;
		//<! Current is blank
		joint[NUSensorsData::AccelerationId] = (joint[NUSensorsData::VelocityId] - m_previous_velocities[i])/delta_t;
		//<! Copy into m_data
		m_data->set(*m_joint_ids[i], m_current_time, joint);
        
        m_previous_positions[i] = joint[NUSensorsData::PositionId];
        m_previous_velocities[i] = joint[NUSensorsData::VelocityId];
	}

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

