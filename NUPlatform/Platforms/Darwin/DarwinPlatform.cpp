/*! @file DarwinPlatform.cpp
    @brief Implementation of DarwinPlatform

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

#include "DarwinPlatform.h"
#include "DarwinCamera.h"
#include "DarwinSensors.h"
#include "DarwinActionators.h"

#include "debug.h"
#include "debugverbositynuplatform.h"
#include "nubotconfig.h"
#include <math.h>

using namespace std;

/*! @brief Constructor for Darwin robotic platform
 */
DarwinPlatform::DarwinPlatform()
{
#if DEBUG_NUPLATFORM_VERBOSITY > 4
    debug << "DarwinPlatform::DarwinPlatform" << endl;
#endif
    init();

    /* Make a list of all of the actionators in the Darwin
        We use a standard way of quickly initialising a vector from a normal array that is initialised
        in its declaration
     */
    // start with the joints
    string temp_servo_names[] = {   string("HeadPitch"), string("HeadYaw"), \
                                    string("LShoulderRoll"), string("LShoulderPitch"), string("LElbowPitch"), \
                                    string("RShoulderRoll"), string("RShoulderPitch"), string("RElbowPitch"), \
                                    string("LHipRoll"),  string("LHipPitch"), string("LHipYaw"), \
									string("LKneePitch"), string("LAnkleRoll"), string("LAnklePitch"), \
                                    string("RHipRoll"),  string("RHipPitch"), string("RHipYaw"), \
									string("RKneePitch"), string("RAnkleRoll"), string("RAnklePitch")};

	int temp_servo_IDs[] = 		{	Robot::JointData::ID_HEAD_TILT, Robot::JointData::ID_HEAD_PAN, \
									Robot::JointData::ID_L_SHOULDER_ROLL, Robot::JointData::ID_L_SHOULDER_PITCH, Robot::JointData::ID_L_ELBOW, \
									Robot::JointData::ID_R_SHOULDER_ROLL, Robot::JointData::ID_R_SHOULDER_PITCH, Robot::JointData::ID_R_ELBOW, \
									Robot::JointData::ID_L_HIP_ROLL, Robot::JointData::ID_L_HIP_PITCH, Robot::JointData::ID_L_HIP_YAW, \
									Robot::JointData::ID_L_KNEE, Robot::JointData::ID_L_ANKLE_ROLL, Robot::JointData::ID_L_ANKLE_PITCH, \
									Robot::JointData::ID_R_HIP_ROLL, Robot::JointData::ID_R_HIP_PITCH, Robot::JointData::ID_R_HIP_YAW, \
									Robot::JointData::ID_R_KNEE, Robot::JointData::ID_R_ANKLE_ROLL, Robot::JointData::ID_R_ANKLE_PITCH};

	float zeros[sizeof(temp_servo_IDs)/sizeof(*temp_servo_IDs)] = {0};

    m_servo_names = vector<string>(temp_servo_names, temp_servo_names + sizeof(temp_servo_names)/sizeof(*temp_servo_names));
	m_servo_IDs = vector<int>(temp_servo_IDs, temp_servo_IDs + sizeof(temp_servo_IDs)/sizeof(*temp_servo_IDs));

	m_servo_Goal_Positions = vector<float>(zeros, zeros + sizeof(zeros)/sizeof(*zeros));
	m_servo_Stiffness = vector<float>(zeros, zeros + sizeof(zeros)/sizeof(*zeros));

	//Code to Connect to Darwin SubController [Taken from Read/Write Tutorial]: 
	linux_cm730 = new Robot::LinuxCM730("/dev/ttyUSB0");
	cm730 = new Robot::CM730(linux_cm730);
	if(cm730->Connect() == false)
	{
		printf("Fail to connect CM-730!\n");
		return;
	}

    #ifdef USE_VISION
        m_camera = new DarwinCamera();
    #else
        m_camera = 0;
    #endif
    m_sensors = new DarwinSensors(this,cm730);
    m_actionators = new DarwinActionators(this,cm730);

	
	//cout << m_servo_Stiffness << endl;
}

DarwinPlatform::~DarwinPlatform()
{
	delete m_camera;
	delete m_sensors;
	delete m_actionators;
	delete cm730;
	delete linux_cm730;
	
}

//Access:: Goal Positions
float DarwinPlatform::getMotorGoalPosition(int localArrayIndex)
{
	return m_servo_Goal_Positions[localArrayIndex];
}


void DarwinPlatform::setMotorGoalPosition(int localArrayIndex, float targetRadians)
{
	m_servo_Goal_Positions[localArrayIndex] = targetRadians;
}

//Access:: Stiffness 
float DarwinPlatform::getMotorStiffness(int localArrayIndex)
{
	return m_servo_Stiffness[localArrayIndex];
}


void DarwinPlatform::setMotorStiffness(int localArrayIndex, float targetStiffness)
{

		int result;

		if( (targetStiffness == 0||isnan(targetStiffness)) && \
			(m_servo_Stiffness[localArrayIndex]!= 0 && !(isnan(m_servo_Stiffness[localArrayIndex]))) )
		{
			//Try to turn stiffness off:			
			//cout << "stiffness off: " << m_servo_names[localArrayIndex] <<"\t" << targetStiffness<<endl;
			result = cm730->WriteByte(m_servo_IDs[localArrayIndex], Robot::MX28::P_TORQUE_ENABLE, 0, 0);
		}

		else if( 	(targetStiffness != 0	&&	!isnan(targetStiffness)) && \
					(m_servo_Stiffness[localArrayIndex] == 0 || isnan(m_servo_Stiffness[localArrayIndex])))
		{
			//Try to turn stiffness on:
			//cout << "stiffness on: " << m_servo_names[localArrayIndex]<<"\t" << targetStiffness<<endl;	
			result = cm730->WriteByte(m_servo_IDs[localArrayIndex], Robot::MX28::P_TORQUE_ENABLE, 1, 0);
		}
		m_servo_Stiffness[localArrayIndex] = targetStiffness;
		//cout << m_servo_Stiffness << endl;
}


