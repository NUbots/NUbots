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

using namespace std;

/*! @brief Constructor for Bear robotic platform
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

	float temp_servo_Offsets[] = 		{	-0.72, 	0.0, \
										//JointID::ID_L_SHOULDER_ROLL, JointID::ID_L_SHOULDER_PITCH, JointID::ID_L_ELBOW,
										-0.7853981, 1.5707963, -1.5707963, \
										//JointID::ID_R_SHOULDER_ROLL, JointID::ID_R_SHOULDER_PITCH, JointID::ID_R_ELBOW,
										0.7853981, -1.5707963, 1.5707963, \
										//JointID::ID_L_HIP_ROLL, JointID::ID_L_HIP_PITCH, JointID::ID_L_HIP_YAW,
										0.0,	0.0,	0.0,	\
										//JointID::ID_L_KNEE, JointID::ID_L_ANKLE_ROLL, JointID::ID_L_ANKLE_PITCH,
										0.0,	0.0,	0.0,	\
										//JointID::ID_R_HIP_ROLL, JointID::ID_R_HIP_PITCH, JointID::ID_R_HIP_YAW, 
										0.0,	0.0,	0.0,	\
										//JointID::ID_R_KNEE, JointID::ID_R_ANKLE_ROLL, JointID::ID_R_ANKLE_PITCH};
										0.0,	0.0,	0.0};

    m_servo_names = vector<string>(temp_servo_names, temp_servo_names + sizeof(temp_servo_names)/sizeof(*temp_servo_names));
	m_servo_IDs = vector<int>(temp_servo_IDs, temp_servo_IDs + sizeof(temp_servo_IDs)/sizeof(*temp_servo_IDs));
	m_servo_Offsets = vector<float>(temp_servo_Offsets, temp_servo_Offsets + sizeof(temp_servo_Offsets)/sizeof(*temp_servo_Offsets));

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

	

}

DarwinPlatform::~DarwinPlatform()
{
	delete m_camera;
	delete m_sensors;
	delete m_actionators;
	delete cm730;
	delete linux_cm730;
	
}

