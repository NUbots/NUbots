/*! @file DarwinWalk.cpp
    @brief Implementation of DarwinWalk class

    @author Aaron Wong
    @author Jason Kulk
    @author Steven Nicklin
 
 Copyright (c) 2009,2011 Jason Kulk, Aaron Wong, Steven Nicklin
 
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

#include "NUPlatform/NUPlatform.h"
#include "NUPlatform/Platforms/Darwin/DarwinJointMapping.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "debug.h"
#include "debugverbositynumotion.h"

//From Darwin Library:
#include <Walking.h>	//Darwin Controller
#include "minIni.h"
#include "MotionStatus.h"

#include <math.h>

/*! @brief Creates a module to walk using Darwin's walk engine

  Also initialises the table used to retrive the joint ids used by the walk engine.
  The walk parameters are read from file and applied.
  The intial walk stance is also set.

  @param data The NUSensorsData class containg sensor data collected from the robot.
  @param actions The NUActionatorsData class used to apply actions for the robot to undertake.
 */
DarwinWalk::DarwinWalk(NUSensorsData* data, NUActionatorsData* actions) :  NUWalk(data, actions)
{
    m_joint_mapping = &DarwinJointMapping::Instance();

    int temp_ids[] =
    {
        Robot::JointData::ID_HEAD_TILT,
        Robot::JointData::ID_HEAD_PAN,
        Robot::JointData::ID_L_SHOULDER_ROLL,
        Robot::JointData::ID_L_SHOULDER_PITCH,
        Robot::JointData::ID_L_ELBOW,
        Robot::JointData::ID_R_SHOULDER_ROLL,
        Robot::JointData::ID_R_SHOULDER_PITCH,
        Robot::JointData::ID_R_ELBOW,
        Robot::JointData::ID_L_HIP_ROLL,
        Robot::JointData::ID_L_HIP_PITCH,
        Robot::JointData::ID_L_HIP_YAW,
        Robot::JointData::ID_L_KNEE,
        Robot::JointData::ID_L_ANKLE_ROLL,
        Robot::JointData::ID_L_ANKLE_PITCH,
        Robot::JointData::ID_R_HIP_ROLL,
        Robot::JointData::ID_R_HIP_PITCH,
        Robot::JointData::ID_R_HIP_YAW,
        Robot::JointData::ID_R_KNEE,
        Robot::JointData::ID_R_ANKLE_ROLL,
        Robot::JointData::ID_R_ANKLE_PITCH
    };
    m_darwin_ids = std::vector<int>(temp_ids, temp_ids + sizeof(temp_ids)/sizeof(*temp_ids));



    // Load the walk settings
    ini = new minIni("config.ini");
    Robot::Walking::GetInstance()->LoadINISettings(ini);
    Robot::Walking::GetInstance()->Initialize();
    Robot::Walking::GetInstance()->PERIOD_TIME = 500;

    m_walk_parameters.load("DarwinWalkDefault");
    //writeParameters();

    // Read in the initial positions from the walk engine.
    std::vector<float> joints(20, 0.0f);
    for(unsigned int id = 0; id < joints.size(); ++id)
    {
        joints[id] = getTarget(id);
    }
    m_initial_larm.assign(joints.begin()+2, joints.begin()+5);
    m_initial_rarm.assign(joints.begin()+5, joints.begin()+8);
    m_initial_lleg.assign(joints.begin()+8, joints.begin()+14);
    m_initial_rleg.assign(joints.begin()+14, joints.begin()+20);
}

/*! @brief Destructor for walk module
 */
DarwinWalk::~DarwinWalk()
{
    delete ini;
}

/*! @brief Calculates and applies the next target position for the walk

  This function is called whenever the walk is active to update the motors target positions.
  The actions to perform the current walk command are calculated and target motor positions are applied.

 */
void DarwinWalk::doWalk()
{

    //SET THE MOTOR POSITIONS IN THE WALK ENGINE:
    updateWalkEngineSensorData();
    //TELL THE WALK ENGINE THE NEW COMMAND
    if(m_speed_x==0  && m_speed_y==0  && m_speed_yaw ==0 )
        Robot::Walking::GetInstance()->Stop();
    else
        Robot::Walking::GetInstance()->Start();

    std::vector<float> speeds = m_walk_parameters.getMaxSpeeds();
    Robot::Walking::GetInstance()->X_MOVE_AMPLITUDE = m_speed_x/speeds[0]*15;
    Robot::Walking::GetInstance()->Y_MOVE_AMPLITUDE = m_speed_y/speeds[1]*30;
    Robot::Walking::GetInstance()->A_MOVE_AMPLITUDE = m_speed_yaw/speeds[2]*25;

    //std::cout << "Walk Commands: " << Robot::Walking::GetInstance()->X_MOVE_AMPLITUDE << " " << Robot::Walking::GetInstance()->Y_MOVE_AMPLITUDE << " " << Robot::Walking::GetInstance()->A_MOVE_AMPLITUDE << std::endl;
    Robot::Walking::GetInstance()->Process();

    //GET THE NEW TARGET POSITIONS FROM THE WALK ENGINE
    updateActionatorsData();
    return;
}

/*! @brief Applies new walk parameters

  Sets the parent member walk parameters and also changes the internal walk parameters
  to match those passed in.
 */
void DarwinWalk::setWalkParameters(const WalkParameters& walkparameters)
{
    m_walk_parameters = walkparameters;
    //update ini
    writeParameters();
}

void DarwinWalk::writeParameters()
{
#if DEBUG_NUMOTION_VERBOSITY > 0
    debug << "DarwinWalk::writeParameters: " << std::endl;
#endif
    std::vector<Parameter>& params = m_walk_parameters.getParameters();
    for(unsigned int i=0; i<params.size(); i++) {
        std::string& nm = params.at(i).name();
        float value = params.at(i).get();
#if DEBUG_NUMOTION_VERBOSITY > 0
    debug << nm << " <- " << value << std::endl;
#endif
        if(nm.compare("x_offset") == 0)
            Robot::Walking::GetInstance()->X_OFFSET = value;
        else if(nm.compare("y_offset") == 0)
            Robot::Walking::GetInstance()->Y_OFFSET = value;
        else if(nm.compare("z_offset") == 0)
            Robot::Walking::GetInstance()->Z_OFFSET = value;
        else if(nm.compare("roll_offset") == 0)
            Robot::Walking::GetInstance()->R_OFFSET = value;
        else if(nm.compare("pitch_offset") == 0)
            Robot::Walking::GetInstance()->P_OFFSET = value;
        else if(nm.compare("yaw_offset") == 0)
            Robot::Walking::GetInstance()->A_OFFSET = value;
        else if(nm.compare("hip_pitch_offset") == 0)
            Robot::Walking::GetInstance()->HIP_PITCH_OFFSET = value;
        else if(nm.compare("period_time") == 0)
            Robot::Walking::GetInstance()->PERIOD_TIME = value;
        else if(nm.compare("dsp_ratio") == 0)
            Robot::Walking::GetInstance()->DSP_RATIO = value;
        else if(nm.compare("step_forward_back_ratio") == 0)
            Robot::Walking::GetInstance()->STEP_FB_RATIO = value;
        else if(nm.compare("foot_height") == 0)
            Robot::Walking::GetInstance()->Z_MOVE_AMPLITUDE = value;
        else if(nm.compare("swing_right_left") == 0)
            Robot::Walking::GetInstance()->Y_SWAP_AMPLITUDE = value;
        else if(nm.compare("swing_top_down") == 0)
            Robot::Walking::GetInstance()->Z_SWAP_AMPLITUDE = value;
        else if(nm.compare("pelvis_offset") == 0)
            Robot::Walking::GetInstance()->PELVIS_OFFSET = value;
        else if(nm.compare("arm_swing_gain") == 0)
            Robot::Walking::GetInstance()->ARM_SWING_GAIN = value;
        else if(nm.compare("balance_knee_gain") == 0)
            Robot::Walking::GetInstance()->BALANCE_KNEE_GAIN = value;
        else if(nm.compare("balance_ankle_pitch_gain") == 0)
            Robot::Walking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN = value;
        else if(nm.compare("balance_hip_roll_gain") == 0)
            Robot::Walking::GetInstance()->BALANCE_HIP_ROLL_GAIN = value;
        else if(nm.compare("balance_ankle_roll_gain") == 0)
            Robot::Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN = value;
        else if(nm.compare("p_gain") == 0)
            Robot::Walking::GetInstance()->P_GAIN = (int)value;
        else if(nm.compare("i_gain") == 0)
            Robot::Walking::GetInstance()->I_GAIN = (int)value;
        else if(nm.compare("d_gain") == 0)
            Robot::Walking::GetInstance()->D_GAIN = (int)value;
        else
            debug << "DarwinWalk::setWalkParameters(): No matching parameter found: " << nm << std::endl;
    }
}

/*! @brief Updates the walk engine with the current sensor data.
 */
void DarwinWalk::updateWalkEngineSensorData()
{
    //Joint Order is same as platform
    static std::vector<float> nu_jointpositions;
    m_data->getPosition(NUSensorsData::All, nu_jointpositions);

    for(unsigned int id = 0; id < nu_jointpositions.size(); ++id)
    {
        setDarwinSensor(id, nu_jointpositions[id]);
    }

    //Update walk engine gyro:
    float VALUETORPS_RATIO = 18.3348;//512/27.925
    float VALUETOACCEL_RATIO = 0.1304; //512/4*981
    std::vector<float> gyro_data(3,0);
    m_data->get(NUSensorsData::Gyro,gyro_data);
    Robot::MotionStatus::FB_GYRO = gyro_data[1]*VALUETORPS_RATIO;
    Robot::MotionStatus::RL_GYRO = gyro_data[0]*VALUETORPS_RATIO;
	
    //Updata WalkEngines Accel Data:
    std::vector<float> accel_data(3,0);
    m_data->get(NUSensorsData::Accelerometer,accel_data);
    Robot::MotionStatus::FB_ACCEL = accel_data[0]*VALUETOACCEL_RATIO;
    Robot::MotionStatus::RL_ACCEL = accel_data[1]*VALUETOACCEL_RATIO;
}

/*! @brief Sets an indivdiual sensor value in the walk engine.
    @param The id of the motor (local).
    @param angle The joint angle in radians.
 */
void DarwinWalk::setDarwinSensor(int id, float angle)
{
    // Note: All of the sensors have been converted to fit our desired position system. i.e. right hand rule about the robots origin reference frame.
    // Because the walk engine is designed to work with the raw sensor values taken from the robot we must convert them back to the original state.
    int value = m_joint_mapping->joint2raw(id, angle);
    Robot::Walking::GetInstance()->m_Joint.SetValue(m_darwin_ids[id],value);
}

/*! @brief Retrieve a indivdiual motor target position from the walk engine
    @param The id of the motor (local).
    @return The target joint position.
 */
float DarwinWalk::getTarget(int id)
{
    // Note: All of the sensors have been converted to fit our desired position system. i.e. right hand rule about the robots origin reference frame.
    // Because the walk engine is designed to work with the raw sensor values taken from the robot we must convert them back to the original state.
    int value = Robot::Walking::GetInstance()->m_Joint.GetValue(m_darwin_ids[id]);
    return m_joint_mapping->raw2joint(id, value);
}

/*! @brief Create the joint commands required to apply the target positions to the motors.
 */
void DarwinWalk::updateActionatorsData()
{
    // the vectors are all static since they are used often and we wish to reduce memory operations.
    static std::vector<float> joints(20, 0.0f);            // All joints
    static std::vector<float> nu_nextLeftArmJoints(m_actions->getSize(NUActionatorsData::LArm), 0.0f);   // Left Arm
    static std::vector<float> nu_nextRightArmJoints(m_actions->getSize(NUActionatorsData::RArm), 0.0f);  // Right Arm
    static std::vector<float> nu_nextLeftLegJoints(m_actions->getSize(NUActionatorsData::LLeg), 0.0f);   // Left Leg
    static std::vector<float> nu_nextRightLegJoints(m_actions->getSize(NUActionatorsData::RLeg), 0.0f);  // Right Leg

    // Retreive all of the joint values
    for(unsigned int id = 0; id < joints.size(); ++id)
    {
        joints[id] = getTarget(id);
    }

    // Assign the values to each effector.
    nu_nextLeftArmJoints.assign(joints.begin()+2, joints.begin()+5);
    nu_nextRightArmJoints.assign(joints.begin()+5, joints.begin()+8);
    nu_nextLeftLegJoints.assign(joints.begin()+8, joints.begin()+14);
    nu_nextRightLegJoints.assign(joints.begin()+14, joints.begin()+20);

    //UPDATE ARMS:
    static std::vector<std::vector<float> >& armgains = m_walk_parameters.getArmGains();
    m_actions->add(NUActionatorsData::RArm, Platform->getTime(), nu_nextRightArmJoints, armgains[0]);
    m_actions->add(NUActionatorsData::LArm, Platform->getTime(), nu_nextLeftArmJoints, armgains[0]);

    //UPDATE LEGS:
    static std::vector<std::vector<float> >& leggains = m_walk_parameters.getLegGains();
    m_actions->add(NUActionatorsData::RLeg, Platform->getTime(), nu_nextRightLegJoints, leggains[0]);
    m_actions->add(NUActionatorsData::LLeg, Platform->getTime(), nu_nextLeftLegJoints, leggains[0]);
    return;
}
