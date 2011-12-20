/*! @file DarwinWalk.cpp
    @brief Implementation of DarwinWalk class

    @author Aaron Wong,  Jason Kulk
 
 Copyright (c) 2009,2011 Jason Kulk, Aaron Wong
 
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

#include "DarwinWalk.h"
//using namespace Kinematics;
#include "minIni.h"
#include "MotionStatus.h"

#include "NUPlatform/NUPlatform.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "debug.h"
#include "debugverbositynumotion.h"


#include <math.h>
#include <list>

//! @todo TODO: put M_PI, NORMALISE, NUM_JOINTS somewhere else
#ifndef M_PI
    #define M_PI 3.1415926535
#endif

template <class T>
inline T NORMALISE(T theta){
    return atan2(sin(theta), cos(theta));
}


/*! Creates a module to walk using Darwin's walk engine
 */
DarwinWalk::DarwinWalk(NUSensorsData* data, NUActionatorsData* actions) :  NUWalk(data, actions)
{
	//Initial Arm Values taken from the walk engine. With Offsets included.
	float L_arms[] = {-0.29, 0.833+1.5707963, -0.5-1.5707963};
	float R_arms[] = {0.29, -0.837-1.5707963, 0.5+1.5707963};	
	float L_legs[] = {0, 0, 0, 0, 0, 0};
	float R_legs[] = {0, 0, 0, 0, 0, 0};
	m_initial_larm = vector<float>(L_arms, L_arms + sizeof(L_arms)/sizeof(*L_arms));
    m_initial_rarm = vector<float>(R_arms, R_arms + sizeof(R_arms)/sizeof(*R_arms));


    m_initial_lleg = vector<float>(L_legs, L_legs + sizeof(L_legs)/sizeof(*L_legs));
    m_initial_rleg = vector<float>(R_legs, R_legs + sizeof(R_legs)/sizeof(*R_legs));

	m_walk_parameters.load("DarwinWalkDefault");


	minIni* ini = new minIni("config.ini");
	Robot::Walking::GetInstance()->LoadINISettings(ini);
	//Robot::Walking::GetInstance()->SaveINISettings(ini);
	Robot::Walking::GetInstance()->Initialize();
	Robot::Walking::GetInstance()->PERIOD_TIME = 500;
}

/*! @brief Destructor for walk module
 */
DarwinWalk::~DarwinWalk()
{
	delete DarwinWalkEngine;
}

void DarwinWalk::doWalk()
{

	//SET THE MOTOR POSITIONS IN THE WALK ENGINE:
	updateWalkEngineSensorData();
	//TELL THE WALK ENGINE THE NEW COMMAND
	if(m_speed_x==0  && m_speed_y==0  && m_speed_yaw ==0 )
		Robot::Walking::GetInstance()->Stop();
	else
		Robot::Walking::GetInstance()->Start();

	vector<float> speeds = m_walk_parameters.getMaxSpeeds();
	Robot::Walking::GetInstance()->X_MOVE_AMPLITUDE = m_speed_x/speeds[0]*15;
	Robot::Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_speed_y/speeds[1]*15;
	Robot::Walking::GetInstance()->A_MOVE_AMPLITUDE = m_speed_yaw/speeds[2]*25;

	//cout << "Walk Commands: " << Robot::Walking::GetInstance()->X_MOVE_AMPLITUDE << " " << Robot::Walking::GetInstance()->Y_MOVE_AMPLITUDE << " " << Robot::Walking::GetInstance()->A_MOVE_AMPLITUDE << endl;
	Robot::Walking::GetInstance()->Process();

	//GET THE NEW TARGET POSITIONS FROM THE WALK ENGINE
	updateActionatorsData();
	return;
}
void DarwinWalk::updateWalkEngineSensorData()
{
	//Joint Order is same as platform
	vector<float> nu_jointpositions;
    m_data->getPosition(NUSensorsData::All, nu_jointpositions);

	//Arms:
	SetDarwinSensor(Robot::JointData::ID_L_SHOULDER_ROLL,	nu_jointpositions[2]);
	SetDarwinSensor(Robot::JointData::ID_L_SHOULDER_PITCH,	nu_jointpositions[3]);
	SetDarwinSensor(Robot::JointData::ID_L_ELBOW,			nu_jointpositions[4]);
	SetDarwinSensor(Robot::JointData::ID_R_SHOULDER_ROLL,	nu_jointpositions[5]);
	SetDarwinSensor(Robot::JointData::ID_R_SHOULDER_PITCH,	nu_jointpositions[6]);
	SetDarwinSensor(Robot::JointData::ID_R_ELBOW,			nu_jointpositions[7]);

	//Leg:
	SetDarwinSensor(Robot::JointData::ID_L_HIP_ROLL,		nu_jointpositions[8]);
	SetDarwinSensor(Robot::JointData::ID_L_HIP_PITCH,		nu_jointpositions[9]);
	SetDarwinSensor(Robot::JointData::ID_L_HIP_YAW,			nu_jointpositions[10]);
	SetDarwinSensor(Robot::JointData::ID_L_KNEE,			nu_jointpositions[11]);
	SetDarwinSensor(Robot::JointData::ID_L_ANKLE_ROLL,		nu_jointpositions[12]);
	SetDarwinSensor(Robot::JointData::ID_L_ANKLE_PITCH,		nu_jointpositions[13]);
	SetDarwinSensor(Robot::JointData::ID_R_HIP_ROLL,		nu_jointpositions[14]);
	SetDarwinSensor(Robot::JointData::ID_R_HIP_PITCH,		nu_jointpositions[15]);
	SetDarwinSensor(Robot::JointData::ID_R_HIP_YAW,			nu_jointpositions[16]);
	SetDarwinSensor(Robot::JointData::ID_R_KNEE,			nu_jointpositions[17]);
	SetDarwinSensor(Robot::JointData::ID_R_ANKLE_ROLL,		nu_jointpositions[18]);
	SetDarwinSensor(Robot::JointData::ID_R_ANKLE_PITCH,		nu_jointpositions[19]);

	//Update walk engine gyro:
	float VALUETORPS_RATIO = 18.3348;//512/27.925
	float VALUETOACCEL_RATIO = 0.1304; //512/4*981
	vector<float> gyro_data(3,0);
	m_data->get(NUSensorsData::Gyro,gyro_data);
	Robot::MotionStatus::FB_GYRO = gyro_data[1]*VALUETORPS_RATIO;
	Robot::MotionStatus::RL_GYRO = gyro_data[0]*VALUETORPS_RATIO;

	//cout << Robot::MotionStatus::FB_GYRO << Robot::MotionStatus::RL_GYRO<< endl;	
	
	//Updata WalkEngines Accel Data:
	vector<float> accel_data(3,0);
	m_data->get(NUSensorsData::Accelerometer,accel_data);
	Robot::MotionStatus::FB_ACCEL = accel_data[0]*VALUETOACCEL_RATIO;
	Robot::MotionStatus::RL_ACCEL = accel_data[1]*VALUETOACCEL_RATIO;

	
}
void DarwinWalk::SetDarwinSensor(int id, float angle)
{
	Robot::Walking::GetInstance()->m_Joint.SetRadian(id,angle);
}
void DarwinWalk::updateActionatorsData()
{
	//UPDATE ARMS:
	static vector<float> nu_nextRightArmJoints(m_actions->getSize(NUActionatorsData::RArm), 1);
	static vector<float> nu_nextLeftArmJoints(m_actions->getSize(NUActionatorsData::LArm), 1);

	vector<vector<float> >& armgains = m_walk_parameters.getArmGains();

	nu_nextRightArmJoints[0] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_SHOULDER_ROLL);
	nu_nextRightArmJoints[1] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_SHOULDER_PITCH)-1.5707963;
	nu_nextRightArmJoints[2] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_ELBOW);

	nu_nextLeftArmJoints[0] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_SHOULDER_ROLL);
	nu_nextLeftArmJoints[1] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_SHOULDER_PITCH)+1.5707963;
	nu_nextLeftArmJoints[2] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_ELBOW);

	m_actions->add(NUActionatorsData::RArm, Platform->getTime(), nu_nextRightArmJoints, armgains[0]);
	m_actions->add(NUActionatorsData::LArm, Platform->getTime(), nu_nextLeftArmJoints, armgains[0]);

	//UPDATE LEGS:
	static vector<float> nu_nextRightLegJoints(m_actions->getSize(NUActionatorsData::RLeg), 0);
	static vector<float> nu_nextLeftLegJoints(m_actions->getSize(NUActionatorsData::LLeg), 0);

	vector<vector<float> >& leggains = m_walk_parameters.getLegGains();

	nu_nextRightLegJoints[0] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_HIP_ROLL);
	nu_nextRightLegJoints[1] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_HIP_PITCH);	
	nu_nextRightLegJoints[2] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_HIP_YAW);
	nu_nextRightLegJoints[3] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_KNEE);
	nu_nextRightLegJoints[4] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_ANKLE_ROLL);
	nu_nextRightLegJoints[5] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_R_ANKLE_PITCH);
	
	nu_nextLeftLegJoints[0] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_HIP_ROLL);
	nu_nextLeftLegJoints[1] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_HIP_PITCH);
	nu_nextLeftLegJoints[2] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_HIP_YAW);
	nu_nextLeftLegJoints[3] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_KNEE);
	nu_nextLeftLegJoints[4] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_ANKLE_ROLL);
	nu_nextLeftLegJoints[5] = Robot::Walking::GetInstance()->m_Joint.GetRadian(Robot::JointData::ID_L_ANKLE_PITCH);
	
	m_actions->add(NUActionatorsData::RLeg, Platform->getTime(), nu_nextRightLegJoints, leggains[0]);
	m_actions->add(NUActionatorsData::LLeg, Platform->getTime(), nu_nextLeftLegJoints, leggains[0]);


	return;
}

