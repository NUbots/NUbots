/*! @file NUSensorsData.cpp
    @brief Implementation of sensor class

    @author Jason Kulk
 
 Copyright (c) 2009 Jason Kulk
 
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

#include "NUSensorsData.h"
#include "debug.h"
#include "debugverbositynusensors.h"

#include <fstream>
#include <cctype>       // for tolower()

NUSensorsData::joint_id_t NUSensorsData::HeadPitch = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::HeadYaw = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::LShoulderRoll = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::LShoulderPitch = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::LElbowRoll = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::LElbowYaw = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::RShoulderRoll = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::RShoulderPitch = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::RElbowRoll = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::RElbowYaw = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::LHipRoll = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::LHipPitch = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::LHipYawPitch = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::LHipYaw = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::LKneePitch = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::LAnkleRoll = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::LAnklePitch = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::RHipRoll = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::RHipPitch = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::RHipYawPitch = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::RHipYaw = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::RKneePitch = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::RAnkleRoll = NUSensorsData::SENSOR_MISSING;
NUSensorsData::joint_id_t NUSensorsData::RAnklePitch = NUSensorsData::SENSOR_MISSING;

/*! @brief Default constructor for NUSensorsData
 */
NUSensorsData::NUSensorsData()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensorsData::NUSensorsData" << endl;
#endif
    
    CurrentTime = 0;
    
    // create the sensor_t's
    addSensor(JointPositions, string("JointPositions"), sensor_t::JOINT_POSITIONS);
    addSensor(JointVelocities, string("JointVelocities"), sensor_t::JOINT_VELOCITIES);
    addSensor(JointAccelerations, string("JointAccelerations"), sensor_t::JOINT_ACCELERATIONS);
    addSensor(JointTargets, string("JointTargets"), sensor_t::JOINT_TARGETS);
    addSensor(JointStiffnesses, string("JointStiffnesses"), sensor_t::JOINT_STIFFNESSES);
    addSensor(JointCurrents, string("JointCurrents"), sensor_t::JOINT_CURRENTS);
    addSensor(JointTorques, string("JointTorques"), sensor_t::JOINT_TORQUES);
    addSensor(JointTemperatures, string("JointTemperatures"), sensor_t::JOINT_TEMPERATURES);

    // Kinematic sensors
    addSoftSensor(LeftLegTransform, string("LeftLegTransform"), sensor_t::KINEMATICS_LEFT_LEG_TRANSFORM);
    addSoftSensor(RightLegTransform, string("RightLegTransform"), sensor_t::KINEMATICS_RIGHT_LEG_TRANSFORM);
    addSoftSensor(SupportLegTransform, string("SupportLegTransform"), sensor_t::KINEMATICS_SUPPORT_LEG_TRANSFORM);
    addSoftSensor(CameraTransform, string("CameraTransform"), sensor_t::KINEMATICS_CAMERA_TRANSFORM);
    addSoftSensor(CameraToGroundTransform, string("CameraToGroundTransform"), sensor_t::KINEMATICS_CAMERA_TO_GROUND_TRANSFORM);

    addSoftSensor(Odometry, string("Odometry"), sensor_t::JOINT_ODOMETRY);
    addSoftSensor(CameraHeight, string("CameraHeight"), sensor_t::JOINT_CAMERAHEIGHT);
    
    // Balance Sensors:
    addSensor(BalanceAccelerometer, string("BalanceAccelerometer"), sensor_t::BALANCE_ACCELEROMETER);
    addSensor(BalanceGyro, string("BalanceGyro"), sensor_t::BALANCE_GYRO);
    addSoftSensor(BalanceGyroOffset, string("BalanceGyroOffset"), sensor_t::BALANCE_GYRO_OFFSET);
    addSoftSensor(BalanceOrientation, string("BalanceOrientation"), sensor_t::BALANCE_ORIENTATION);
    addSoftSensor(BalanceHorizon, string("BalanceHorzion"), sensor_t::BALANCE_HORIZON);
    addSoftSensor(BalanceZMP, string("BalanceZMP"), sensor_t::BALANCE_ZMP);    
    addSoftSensor(BalanceFalling, string("BalanceFalling"), sensor_t::BALANCE_FALLING);
    addSoftSensor(BalanceFallen, string("BalanceFallen"), sensor_t::BALANCE_FALLEN);
    
    // Distance Sensors:
    addSensor(DistanceLeftValues, string("DistanceLeftValues"), sensor_t::DISTANCE_LEFT_VALUES);
    addSensor(DistanceRightValues, string("DistanceRightValues"), sensor_t::DISTANCE_RIGHT_VALUES);
    
    // Foot Pressure Sensors:
    addSensor(FootSoleValues, string("FootSoleValues"), sensor_t::FOOT_SOLE_VALUES);
    addSensor(FootBumperValues, string("FootBumperValues"), sensor_t::FOOT_BUMPER_VALUES);
    addSoftSensor(FootCoP, string("FootCoP"), sensor_t::FOOT_COP);
    addSoftSensor(FootForce, string("FootForce"), sensor_t::FOOT_FORCE);
    addSoftSensor(FootSupport, string("FootSupport"), sensor_t::FOOT_SUPPORT);
    addSoftSensor(FootContact, string("FootContact"), sensor_t::FOOT_CONTACT);
    addSoftSensor(FootImpact, string("FootImpact"), sensor_t::FOOT_IMPACT);
    
    // Buttons Sensors:
    addSensor(ButtonValues, string("ButtonValues"), sensor_t::BUTTON_VALUES);
    addSoftSensor(ButtonTriggers, string("ButtonTriggers"), sensor_t::BUTTON_TRIGGERS);
    
    // Battery Sensors:
    addSensor(BatteryValues, string("BatteryValues"), sensor_t::BATTERY_VALUES);
    
    // Motion Sensors:
    addSensor(MotionFallActive, string("MotionFallActive"), sensor_t::MOTION_FALL_ACTIVE);
    addSensor(MotionGetupActive, string("MotionGetupActive"), sensor_t::MOTION_GETUP_ACTIVE);
    addSensor(MotionKickActive, string("MotionKickActive"), sensor_t::MOTION_KICK_ACTIVE);
    addSensor(MotionSaveActive, string("MotionSaveActive"), sensor_t::MOTION_SAVE_ACTIVE);
    addSensor(MotionScriptActive, string("MotionScriptActive"), sensor_t::MOTION_SCRIPT_ACTIVE);
    addSensor(MotionWalkSpeed, string("MotionWalkSpeed"), sensor_t::MOTION_WALK_SPEED);
    addSensor(MotionWalkMaxSpeed, string("MotionWalkMaxSpeed"), sensor_t::MOTION_WALK_MAX_SPEED);
    addSensor(MotionHeadCompletionTime, string("MotionHeadCompletionTime"), sensor_t::MOTION_HEAD_COMPLETION_TIME);

    // GPS Sensor:
    addSensor(GPS, string("GPS"), sensor_t::GPS_VALUES);
    addSensor(Compass, string("Compass"), sensor_t::COMPASS_VALUES);
}

/*! @brief Adds a sensor to the class
    @param p_sensor a pointer that will be updated to point to the new sensor
    @param sensorname the name of the sensor
    @param sensorid the id of the sensor's type (eg. sensor_t::JOINT_POSITIONS)
 */
void NUSensorsData::addSensor(sensor_t*& p_sensor, string sensorname, sensor_t::sensor_id_t sensorid)
{
    p_sensor = new sensor_t(sensorname, sensorid);
    m_sensors.push_back(p_sensor);
}

/*! @brief Adds a soft sensor to the class
 @param p_sensor a pointer that will be updated to point to the new sensor
 @param sensorname the name of the sensor
 @param sensorid the id of the sensor's type (eg. sensor_t::JOINT_POSITIONS)
 */
void NUSensorsData::addSoftSensor(sensor_t*& p_sensor, string sensorname, sensor_t::sensor_id_t sensorid)
{
    p_sensor = new sensor_t(sensorname, sensorid, true);
    m_sensors.push_back(p_sensor);
}

NUSensorsData::~NUSensorsData()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensorsData::~NUSensorsData" << endl;
#endif
    for (size_t i=0; i<m_sensors.size(); i++)
        delete m_sensors[i];
}

/******************************************************************************************************************************************
                                                                                                                                Get Methods
 ******************************************************************************************************************************************/

/*! @brief Gets the requested joint position. If the operation is successful true is returned, 
           otherwise false is returned and position is unchanged
    @param jointid the id of the joint you want the data for
    @param position the position will be placed in this variable
 */
bool NUSensorsData::getJointPosition(joint_id_t jointid, float& position)
{
    return getJointData(JointPositions, jointid, position);
}

/*! @brief Gets the requested joint velocity. If the operation is successful true is returned, 
           otherwise false is returned and velocity is unchanged
    @param jointid the id of the joint you want the data for
    @param velocity the velocity will be placed in this variable
 */
bool NUSensorsData::getJointVelocity(joint_id_t jointid, float& velocity)
{
    return getJointData(JointVelocities, jointid, velocity);
}

/*! @brief Gets the requested joint acceleration. If the operation is successful true is returned, 
           otherwise false is returned and acceleration is unchanged
    @param jointid the id of the joint you want the data for
    @param acceleration the acceleration will be placed in this variable
 */
bool NUSensorsData::getJointAcceleration(joint_id_t jointid, float& acceleration)
{
    return getJointData(JointAccelerations, jointid, acceleration);
}

/*! @brief Gets the requested joint target. If the operation is successful true is returned, 
           otherwise false is returned and target is unchanged
    @param jointid the id of the joint you want the data for
    @param target the target will be placed in this variable
 */
bool NUSensorsData::getJointTarget(joint_id_t jointid, float& target)
{
    return getJointData(JointTargets, jointid, target);
}

/*! @brief Gets the requested joint stiffness. If the operation is successful true is returned, 
           otherwise false is returned and stiffness is unchanged
    @param jointid the id of the joint you want the data for
    @param stiffness the stiffness will be placed in this variable
 */
bool NUSensorsData::getJointStiffness(joint_id_t jointid, float& stiffness)
{
    return getJointData(JointStiffnesses, jointid, stiffness);
}

/*! @brief Gets the requested joint current. If the operation is successful true is returned, 
           otherwise false is returned and current is unchanged
    @param jointid the id of the joint you want the data for
    @param current the current will be placed in this variable
 */
bool NUSensorsData::getJointCurrent(joint_id_t jointid, float& current)
{
    return getJointData(JointCurrents, jointid, current);
}

/*! @brief Gets the requested joint torque. If the operation is successful true is returned, 
           otherwise false is returned and torque is unchanged
    @param jointid the id of the joint you want the data for
    @param torque the torque will be placed in this variable
 */
bool NUSensorsData::getJointTorque(joint_id_t jointid, float& torque)
{
    return getJointData(JointTorques, jointid, torque);
}

/*! @brief Gets the requested joint temperatures. If the operation is successful true is returned, 
           otherwise false is returned and temperatures is unchanged
    @param jointid the id of the joint you want the data for
    @param temperature the temperature will be placed in this variable
 */
bool NUSensorsData::getJointTemperature(joint_id_t jointid, float& temperature)
{
    return getJointData(JointTemperatures, jointid, temperature);
}

/*! @brief Returns the number of joints in the specified body part
 @param partid the id of the body part
 @return the number of joints
 */
int NUSensorsData::getNumberOfJoints(bodypart_id_t partid)
{
    if (partid == AllJoints)
        return m_num_joints;
    else if (partid == BodyJoints)
        return m_num_body_joints;
    else if (partid == HeadJoints)
        return m_num_head_joints;
    else if (partid == LeftArmJoints)
        return m_num_arm_joints;
    else if (partid == RightArmJoints)
        return m_num_arm_joints;
    else if (partid == TorsoJoints)
        return m_num_torso_joints;
    else if (partid == LeftLegJoints)
        return m_num_leg_joints;
    else if (partid == RightLegJoints)
        return m_num_leg_joints;
    else
    {
        debug << "NUSensorsData::getNumberOfJoints. UNDEFINED Body part.";
        return 0;
    }
}

/*! @brief Gets the requested joint positions in a given body part. If the get is successful true is returned
 otherwise false is returned, and the input variable is left unchanged.
 @param bodypart the id of the body part to want the data for
 @param positions the input vector that will be updated to contain the requested data
 */
bool NUSensorsData::getJointPositions(bodypart_id_t bodypart, vector<float>& positions)
{
    return getJointsData(JointPositions, bodypart, positions);
}

/*! @brief Gets the requested joint velocities in a given body part. If the get is successful true is returned
 otherwise false is returned, and the input variable is left unchanged.
 @param bodypart the id of the body part to want the data for
 @param velocities the input vector that will be updated to contain the requested data
 */
bool NUSensorsData::getJointVelocities(bodypart_id_t bodypart, vector<float>& velocities)
{
    return getJointsData(JointVelocities, bodypart, velocities);
}

/*! @brief Gets the requested joint accelerations in a given body part. If the get is successful true is returned
 otherwise false is returned, and the input variable is left unchanged.
 @param bodypart the id of the body part to want the data for
 @param accelerations the input vector that will be updated to contain the requested data
 */
bool NUSensorsData::getJointAccelerations(bodypart_id_t bodypart, vector<float>& accelerations)
{
    return getJointsData(JointAccelerations, bodypart, accelerations);
}

/*! @brief Gets the requested joint targets in a given body part. If the get is successful true is returned
 otherwise false is returned, and the input variable is left unchanged.
 @param bodypart the id of the body part to want the data for
 @param targets the input vector that will be updated to contain the requested data
 */
bool NUSensorsData::getJointTargets(bodypart_id_t bodypart, vector<float>& targets)
{
    return getJointsData(JointTargets, bodypart, targets);
}

/*! @brief Gets the requested joint stiffnesses in a given body part. If the get is successful true is returned
 otherwise false is returned, and the input variable is left unchanged.
 @param bodypart the id of the body part to want the data for
 @param stiffnesses the input vector that will be updated to contain the requested data
 */
bool NUSensorsData::getJointStiffnesses(bodypart_id_t bodypart, vector<float>& stiffnesses)
{
    return getJointsData(JointStiffnesses, bodypart, stiffnesses);
}

/*! @brief Gets the requested joint currents in a given body part. If the get is successful true is returned
 otherwise false is returned, and the input variable is left unchanged.
 @param bodypart the id of the body part to want the data for
 @param currents the input vector that will be updated to contain the requested data
 */
bool NUSensorsData::getJointCurrents(bodypart_id_t bodypart, vector<float>& currents)
{
    return getJointsData(JointCurrents, bodypart, currents);
}

/*! @brief Gets the requested joint torques in a given body part. If the get is successful true is returned
 otherwise false is returned, and the input variable is left unchanged.
 @param bodypart the id of the body part to want the data for
 @param torques the input vector that will be updated to contain the requested data
 */
bool NUSensorsData::getJointTorques(bodypart_id_t bodypart, vector<float>& torques)
{
    return getJointsData(JointTorques, bodypart, torques);
}

/*! @brief Gets the requested joint temperatures in a given body part. If the get is successful true is returned
 otherwise false is returned, and the input variable is left unchanged.
 @param bodypart the id of the body part to want the data for
 @param temperatures the input vector that will be updated to contain the requested data
 */
bool NUSensorsData::getJointTemperatures(bodypart_id_t bodypart, vector<float>& temperatures)
{
    return getJointsData(JointTemperatures, bodypart, temperatures);
}

/* @brief Gets the transform matrix of the left leg
   @param value will be updated with the left leg transform
 */
bool NUSensorsData::getLeftLegTransform(Matrix& value)
{
    if (LeftLegTransform == NULL || LeftLegTransform->IsValid == false)
        return false;
    else
    {
        value = Matrix4x4fromVector(LeftLegTransform->Data);
        return true;
    }
}

/* @brief Gets the transform matrix of the right leg
   @param value will be updated with the right leg transform
 */
bool NUSensorsData::getRightLegTransform(Matrix& value)
{
    if (RightLegTransform == NULL || RightLegTransform->IsValid == false)
        return false;
    else
    {
        value = Matrix4x4fromVector(RightLegTransform->Data);
        return true;
    }
}

/* @brief Gets the transform matrix of the support leg
   @param value will be updated with the support leg transform
 */
bool NUSensorsData::getSupportLegTransform(Matrix& value)
{
    if (SupportLegTransform == NULL || SupportLegTransform->IsValid == false)
        return false;
    else
    {
        value = Matrix4x4fromVector(SupportLegTransform->Data);
        return true;
    }
}

/* @brief Gets the transform matrix of the camera
   @param value will be updated with the camera transform
 */
bool NUSensorsData::getCameraTransform(Matrix& value)
{
    if (CameraTransform == NULL || CameraTransform->IsValid == false)
        return false;
    else
    {
        value = Matrix4x4fromVector(CameraTransform->Data);
        return true;
    }
}

/* @brief Gets the transform matrix converting from the camera coordinates to ground based coordinates
   @param value will be updated with the camera to ground transform
 */
bool NUSensorsData::getCameraToGroundTransform(Matrix& value)
{
    if (CameraToGroundTransform == NULL || CameraToGroundTransform->IsValid == false)
        return false;
    else
    {
        value = Matrix4x4fromVector(CameraToGroundTransform->Data);
        return true;
    }
}

/*! @brief Gets the odometry data since the last call
    @param time the time of the last call
    @param values will be updated with the odometry [x (cm), y(cm), yaw(rad)] since time
 */
bool NUSensorsData::getOdometry(float& time, vector<float>& values)
{
    static double timeoflastcall = 0;
    if (Odometry == NULL || Odometry->IsValid == false)
        return false;
    else
    {
        time = timeoflastcall;
        values = Odometry->Data;
        timeoflastcall = CurrentTime;
        Odometry->Data = vector<float> (Odometry->size(),0);
        return true;
    }
}

/* @brief Gets the height of the camera off the ground in cm
   @param height will be updated with the height of the camera from the ground
 */
bool NUSensorsData::getCameraHeight(float& height)
{
    if (CameraHeight == NULL || CameraHeight->IsValid == false)
        return false;
    else
    {
        height = CameraHeight->Data[0];
        return true;
    }
}

/*! @brief Gets the names of the joints in a particular body part.
    @param partid the id of the body part 
    @param names will be updated to contain the names of the joints in that body part
 */
bool NUSensorsData::getJointNames(bodypart_id_t partid, vector<string>& names)
{
    vector<joint_id_t> selectedjoints;
    if (partid == AllJoints)
        selectedjoints = m_all_joint_ids;
    if (partid == BodyJoints)
        selectedjoints = m_body_ids;
    else if (partid == HeadJoints)
        selectedjoints = m_head_ids;
    else if (partid == LeftArmJoints)
        selectedjoints = m_larm_ids;
    else if (partid == RightArmJoints)
        selectedjoints = m_rarm_ids;
    else if (partid == TorsoJoints)
        selectedjoints = m_torso_ids;
    else if (partid == LeftLegJoints)
        selectedjoints = m_lleg_ids;
    else if (partid == RightLegJoints)
        selectedjoints = m_rleg_ids;
    else
    {
        errorlog << "NUSensorsData::getJointNames. UNDEFINED Body part.";
        return false;
    }
    
    names.clear();
    for (unsigned int i=0; i<selectedjoints.size(); i++)
        names.push_back(m_joint_names[selectedjoints[i]]);
    return true;
}

/* The grunt work for getting single joint data values
    @param p_sensor a pointer to the sensor from which the value will come
    @param jointid the unique joint id
    @param data the variable that will be updated with the new value
 */
bool NUSensorsData::getJointData(sensor_t* p_sensor, joint_id_t jointid, float& data)
{
    if (jointid == NUSensorsData::SENSOR_MISSING || p_sensor->IsValid == false)
        return false;
    else
    {
        data = p_sensor->Data[jointid];
        return true;
    }
}

/* The grunt work for getting joint vector data.
    @param p_sensor a pointer to the sensor from which we are going to get the vector of data from
    @param partid the id of the body part to get the vector of data
    @param data the variable that will be updated to have the vector of data
 */
bool NUSensorsData::getJointsData(sensor_t* p_sensor, bodypart_id_t partid, vector<float>& data)
{
    if (p_sensor == NULL || p_sensor->IsValid == false)
        return false;
    else if (partid == AllJoints)
    {   // if we want all joints then it is easy
        data = p_sensor->Data;
        return true;
    }
    else 
    {   // if we want a subset, then its harder; use the ids lists to make a subarray
        vector<joint_id_t> selectedjoints;
        if (partid == BodyJoints)
            selectedjoints = m_body_ids;
        else if (partid == HeadJoints)
            selectedjoints = m_head_ids;
        else if (partid == LeftArmJoints)
            selectedjoints = m_larm_ids;
        else if (partid == RightArmJoints)
            selectedjoints = m_rarm_ids;
        else if (partid == TorsoJoints)
            selectedjoints = m_torso_ids;
        else if (partid == LeftLegJoints)
            selectedjoints = m_lleg_ids;
        else if (partid == RightLegJoints)
            selectedjoints = m_rleg_ids;
        else
        {
            debug << "NUSensorsData::getNumberOfJoints. UNDEFINED Body part.";
            return false;
        }

        data.clear();
        data.reserve(selectedjoints.size());
        for (unsigned int i=0; i<selectedjoints.size(); i++)
            data.push_back(p_sensor->Data[selectedjoints[i]]);
        return true;
    }
    return true;
}

/*! @brief Gets the accelerometer values [ax, ay, az] in cm/s/s
    @param values will be updated with the current accelerometer readings
 */
bool NUSensorsData::getAccelerometerValues(vector<float>& values)
{
    if (BalanceAccelerometer == NULL || BalanceAccelerometer->IsValid == false)
        return false;
    else
    {
        values = BalanceAccelerometer->Data;
        return true;
    }
}

/*! @brief Gets the accelerometer values [ax, ay, az] in cm/s/s
    @param values will be updated with the current accelerometer readings
 */
bool NUSensorsData::getHorizon(vector<float>& values)
{
    if (BalanceHorizon == NULL || BalanceHorizon->IsValid == false)
        return false;
    else
    {
        values = BalanceHorizon->Data;
        return true;
    }
}

/*! @brief Gets the last button trigger times for the chest button, left
            foot bumper and right foot bumper.
    @param values will be updated with the current utton trigger times.
 */
bool NUSensorsData::getButtonTriggers(vector<float>& values)
{
    if (ButtonTriggers == NULL || ButtonTriggers->IsValid == false)
        return false;
    else
    {
        values = ButtonTriggers->Data;
        return true;
    }
}

/*! @brief Gets the gyro values [gx, gy, gz] in rad/s
    @param values will be updated with the current gyro readings
 */
bool NUSensorsData::getGyroValues(vector<float>& values)
{
    if (BalanceGyro == NULL || BalanceGyro->IsValid == false)
        return false;
    else
    {
        values = BalanceGyro->Data;
        return true;
    }
}

/*! @brief Gets the gyro offset values [offsetx, offsety, offsetz]
    @param values will be updated with the current estimate of the gyro offset
    @return returns true if the values are valid false otherwise
 */
bool NUSensorsData::getGyroOffsetValues(vector<float>& values)
{
    if (BalanceGyroOffset == NULL || BalanceGyroOffset->IsValid == false)
        return false;
    else
    {
        values = BalanceGyroOffset->Data;
        return true;
    }
}

/*! @brief Gets the gyro values after the offset and filtering is applied [gx, gy, gz]
    @param values will be updated with the current estimate of the gyro
    @return returns true if the values are valid false otherwise
 */
bool NUSensorsData::getGyroFilteredValues(vector<float>& values)
{
    if (BalanceGyro == NULL || BalanceGyro->IsValid == false || BalanceGyroOffset == NULL || BalanceGyroOffset->IsValid == false)
        return false;
    else
    {
        values = vector<float>(3,0);
        for (size_t i=0; i<values.size(); i++)
            values[i] = BalanceGyro->Data[i] - BalanceGyroOffset->Data[i];
        return true;
    }
}

/*! @brief Gets the orientation [roll, pitch, yaw] in radians of the robot's torso
    @param values will be updated with the current orientation estimate
 */
bool NUSensorsData::getOrientation(vector<float>& values)
{
    if (BalanceOrientation == NULL || BalanceOrientation->IsValid == false)
        return false;
    else 
    {
        values = BalanceOrientation->Data;
        return true;
    }
}

/*! @brief Gets the zero moment point [x,y] in cm from somewhere?
    @param values will be updated with the current ZMP estimate
 */
bool NUSensorsData::getZMP(vector<float>& values)
{
    if (BalanceZMP == NULL || BalanceZMP->IsValid == false)
        return false;
    else 
    {
        values = BalanceZMP->Data;
        return true;
    }
}

/*! @brief Gets the falling sense [sum, left, right, forward, backward] 
    @param values will be updated with the current falling measurements [sum, left, right, forward, backward]
 */
bool NUSensorsData::getFalling(vector<float>& values)
{
    if (BalanceFalling == NULL || BalanceFalling->IsValid == false)
        return false;
    else 
    {
        values = BalanceFalling->Data;
        return true;
    }
}

/*! @brief Gets the fallen sense [sum, left, right, forward, backward] 
    @param values will be updated with the current fallen measurements [sum, left, right, forward, backward]
 */
bool NUSensorsData::getFallen(vector<float>& values)
{
    if (BalanceFallen == NULL || BalanceFallen->IsValid == false)
        return false;
    else 
    {
        values = BalanceFallen->Data;
        return true;
    }
}

/*! @brief Gets the distance sensor readings (sensors from left to right) in centimeters
    @param values will be updated with the current distance readings
 */
bool NUSensorsData::getDistanceLeftValues(vector<float>& values)
{
    if (DistanceLeftValues == NULL || DistanceLeftValues->IsValid == false)
        return false;
    else
    {
        values = DistanceLeftValues->Data;
        return true;
    }
}

bool NUSensorsData::getDistanceRightValues(vector<float>& values)
{
    if (DistanceRightValues == NULL || DistanceRightValues->IsValid == false)
        return false;
    else
    {
        values = DistanceRightValues->Data;
        return true;
    }
}

/*! @brief Gets the battery readings [voltage (V), current (A), charge (%)]
    @param values will be updated with the current battery sensor values
 */
bool NUSensorsData::getBatteryValues(vector<float>& values)
{
    if (BatteryValues == NULL || BatteryValues->IsValid == false)
        return false;
    else
    {
        values = BatteryValues->Data;
        return true;
    }
}

/*! @brief Gets the GPS readings [x (cm), y(cm), z (cm)]
    @param values will be updated with the gps coordinates of the robot [x (cm), y (cm), z (cm)]
 */
bool NUSensorsData::getGPSValues(vector<float>& values)
{
    if (GPS == NULL || GPS->IsValid == false)
        return false;
    else
    {
        values = GPS->Data;
        return true;
    }
}

/*! @brief Gets the compass reading [heading]
    @param values will be updated with the compass reading
 */
bool NUSensorsData::getCompassValues(vector<float>& values)
{
    if (Compass == NULL || Compass->IsValid == false)
        return false;
    else
    {
        values = Compass->Data;
        return true;
    }
}

/*! @brief Gets the foot sole pressure sensor values (order: left to right front to back) in Newtons
    @param footid the id of the part of the foot you want the readings for
    @param values will be updated with the current readings for the selected foot
 */
bool NUSensorsData::getFootSoleValues(foot_id_t footid, vector<float>& values)
{
    if (FootSoleValues == NULL || FootSoleValues->IsValid == false)
        return false;
    else
    {
        int numfootsolesensors = FootSoleValues->Data.size();
        if (footid == AllFeet)
            values = FootSoleValues->Data;
        else if (footid == LeftFoot)
        {
            vector<float> leftfootvalues(numfootsolesensors/2, 0);
            for (unsigned int i=0; i<leftfootvalues.size(); i++)
                leftfootvalues[i] = FootSoleValues->Data[i];
            values = leftfootvalues;
        }
        else if (footid == RightFoot)
        {
            vector<float> rightfootvalues(numfootsolesensors/2, 0);
            for (unsigned int i=0; i<rightfootvalues.size(); i++)
                rightfootvalues[i] = FootSoleValues->Data[i + numfootsolesensors/2];
            values = rightfootvalues;
        }
        else
        {
            debug << "NUSensorsData::getFootSoleValues(). Unknown foot id." << endl;
            return false;
        }
        return true;
    }
}

/*! @brief Gets the centre of pressure as measured by feet sensors
 
    If a single foot is requested the x and y positions are relative to the position of the ankle on the foot.
    If the CoP for both feet is requested the x and y positions are relative to the torso
 
    @param footid LeftFoot, RightFoot, AllFeet
    @param x the distance in cm forwards 
    @param y the distance in cm backwards
 */
bool NUSensorsData::getFootCoP(foot_id_t footid, float& x, float& y)
{
    if (FootCoP == NULL || FootCoP->IsValid == false)
        return false;
    else
    {
        if (footid == LeftFoot)
        {
            x = (*FootCoP)[0];
            y = (*FootCoP)[1];
        }
        else if (footid == RightFoot)
        {
            x = (*FootCoP)[2];
            y = (*FootCoP)[3];
        }
        else if (footid == AllFeet)
        {
            x = (*FootCoP)[4];
            y = (*FootCoP)[5];
        }
        else
        {
            debug << "NUSensorsData::getFootForce(). Unknown foot id." << endl;
            return false;
        }
        return true;
    }
}

/*! @brief Gets the foot bumper sensor values (order: left to right) in binary (0=off 1=on)
    @param footid the id of the part of the foot you want the readings for
    @param values will be updated with the current readings for the selected foot
 */
bool NUSensorsData::getFootBumperValues(foot_id_t footid, vector<float>& values)
{
    if (FootBumperValues == NULL || FootBumperValues->IsValid == false)
        return false;
    else
    {
        int numfootbumpersensors = FootBumperValues->Data.size();
        if (footid == AllFeet)
            values = FootBumperValues->Data;
        else if (footid == LeftFoot)
        {
            vector<float> leftfootvalues(numfootbumpersensors/2, 0);
            for (unsigned int i=0; i<leftfootvalues.size(); i++)
                leftfootvalues[i] = FootBumperValues->Data[i];
            values = leftfootvalues;
        }
        else if (footid == RightFoot)
        {
            vector<float> rightfootvalues(numfootbumpersensors/2, 0);
            for (unsigned int i=0; i<rightfootvalues.size(); i++)
                rightfootvalues[i] = FootBumperValues->Data[i + numfootbumpersensors/2];
            values = rightfootvalues;
        }
        else
        {
            debug << "NUSensorsData::getFootBumperValues(). Unknown foot id." << endl;
            return false;
        }
        return true;
    }
}

/*! @brief Gets the total force on the foot in Newtons
    @param footid the id of the part of the foot you want the readings for
    @param force will be updated with the current readings for the selected foot
 */
bool NUSensorsData::getFootForce(foot_id_t footid, float& force)
{
    force = 0;
    if (FootForce == NULL || FootForce->IsValid == false)
        return false;
    else
    {
        if (footid == LeftFoot)
        {
            force = (*FootForce)[0];
        }
        else if (footid == RightFoot)
        {
            force = (*FootForce)[1];
        }
        else if (footid == AllFeet)
        {
            force = (*FootForce)[2];
        }
        else
        {
            debug << "NUSensorsData::getFootForce(). Unknown foot id." << endl;
            force = 0;
            return false;
        }
        return true;
    }
}

/*! @brief Gets the whether footid is in contact with the ground
    @param footid the id of the foot
    @param contact will be updated to true if the foot is on the ground, false if it is not on the ground
 */
bool NUSensorsData::getFootContact(foot_id_t footid, bool& contact)
{
    if (FootContact == NULL || FootContact->IsValid == false)
        return false;
    else
    {
        if (footid == LeftFoot)
            contact = (*FootContact)[0];
        else if (footid == RightFoot)
            contact = (*FootContact)[1];
        else if (footid == AllFeet)
            contact = (*FootContact)[2];
        else
        {
            debug << "NUSensorsData::getFootContact(). Unknown foot id." << endl;
            return false;
        }
        return true;
    }
}

/*! @brief Gets the total force on the foot in Newtons
    @param footid the id of the part of the foot you want the readings for
    @param support will be updated to true if footid is supporting the robot.
    @return true if support was updated, false if invalid
 */
bool NUSensorsData::getFootSupport(foot_id_t footid, bool& support)
{
    if (FootSupport == NULL || FootSupport->IsValid == false)
        return false;
    else
    {
        if (footid == LeftFoot)
            support = static_cast<bool> ((*FootSupport)[0]);
        else if (footid == RightFoot)
            support = static_cast<bool> ((*FootSupport)[1]);
        else if (footid == AllFeet)
            support = static_cast<bool> ((*FootSupport)[0]) and static_cast<bool> ((*FootSupport)[1]);
        else
        {
            debug << "NUSensorsData::getFootForce(). Unknown foot id." << endl;
            return false;
        }
        return true;
    }
}

/*! @brief Gets the button values (order: importance) in binary (0=off 1=on)
    @param buttonid the id of the button(s) you want the readings for
    @param values will be updated with the current readings for the selected button(s)
 */
bool NUSensorsData::getButtonValues(button_id_t buttonid, vector<float>& values)
{
    if (ButtonValues == NULL || ButtonValues->IsValid == false)
        return false;
    else
    {
        if (buttonid == AllButtons)
            values = ButtonValues->Data;
        else if (buttonid == MainButton)
            values = vector<float> (1, ButtonValues->Data[0]);
        else if (buttonid == SecondaryButton)
        {
            if (ButtonValues->Data.size() > 1)
                values = vector<float> (1, ButtonValues->Data[1]);
            else
                return false;
        }
        else
        {
            debug << "NUSensorsData::getButtonValues(). Unknown button id." << endl;
            return false;
        }
        return true;
    }
}

/******************************************************************************************************************************************
                                                                                                                 Convienent sub-get Methods
 ******************************************************************************************************************************************/

/*! @brief Returns true if the robot is falling over, false if it is not falling (or it is impossble to tell)
 */
bool NUSensorsData::isFalling()
{
    if (BalanceFalling == NULL || BalanceFalling->IsValid == false)       // if there is no balance sensor it is impossible to tell it is falling over
        return false;       
    else if (BalanceFalling->Data[0] > 0)
        return true;
    else
        return false;
}

/*! @brief Returns true if the robot has fallen over, false if it hasn't (or it is impossible to tell)
 */
bool NUSensorsData::isFallen()
{
    if (BalanceFallen == NULL || BalanceFallen->IsValid == false)       // if there is no balance sensor it is impossible to tell it has fallen over
        return false;       
    else if (BalanceFallen->Data[0] > 0)
        return true;
    else
        return false;
}

/*! @brief Returns true if the robot is on the ground, false otherwise
 */
bool NUSensorsData::isOnGround()
{
    if (FootContact == NULL || FootContact->IsValid == false)
        return true;
    else if (FootContact->Data[2] > 0)
        return true;
    else
        return false;
}

/*! @brief Returns true if the robot is currently incapacitated 
 */
bool NUSensorsData::isIncapacitated()
{
    bool gettingup = false;
    getMotionGetupActive(gettingup);
    return isFalling() or isFallen() or not isOnGround() or gettingup;
}

/*! @brief Returns true has impacted in the ground in this cycle
    @param footid the foot you want to know about
    @param time time will be updated with the time at which the last impact on that foot occured.
    @return true if the foot hit the ground in *this* cycle, it will be false otherwise (ie it will be false the cycle after the impact; that is what the time is for ;))
 */
bool NUSensorsData::footImpact(foot_id_t footid, float& time)
{
    if (FootImpact == NULL || FootImpact->IsValid == false)
    {
        time = 0;
        return false;
    }
    else if (footid == LeftFoot)
        time = FootImpact->Data[0];
    else if (footid == RightFoot)
        time = FootImpact->Data[1];
    else if (footid == AllFeet)
    {
        if (FootImpact->Data[0] > FootImpact->Data[1])              // left impact was most recent, so return it
            time = FootImpact->Data[0];
        else
            time = FootImpact->Data[1];
    }
    else
        return false;
        
    
    if (CurrentTime - time <= 0.1) 
        return true;
    else
        return false;

}

/*! @brief Get whether the fall motion module is active
    @param active will be updated to true if the fall module is active
    @return true if the data is valid
 */
bool NUSensorsData::getMotionFallActive(bool& active)
{
    if (not MotionFallActive->IsValid)
        return false;
    else
    {
        active = MotionFallActive->Data[0];
        return true;
    }
}

/*! @brief Get whether the getup motion module is active
    @param active will be updated to true if the getup module is active
    @return true if the data is valid
 */
bool NUSensorsData::getMotionGetupActive(bool& active)
{
    if (not MotionGetupActive->IsValid)
        return false;
    else
    {
        active = MotionGetupActive->Data[0];
        return true;
    }
}

/*! @brief Get whether the kick motion module is active
    @param active will be updated to true if the kick module is active
    @return true if the data is valid
 */
bool NUSensorsData::getMotionKickActive(bool& active)
{
    if (not MotionKickActive->IsValid)
        return false;
    else
    {
        active = MotionKickActive->Data[0];
        return true;
    }
}

/*! @brief Get whether the save motion module is active
    @param active will be updated to true if the save module is active
    @return true if the data is valid
 */
bool NUSensorsData::getMotionSaveActive(bool& active)
{
    if (not MotionSaveActive->IsValid)
        return false;
    else
    {
        active = MotionSaveActive->Data[0];
        return true;
    }
}

/*! @brief Get whether the script motion module is active
    @param active will be updated to true if the script module is active
    @return true if the data is valid
 */
bool NUSensorsData::getMotionScriptActive(bool& active)
{
    if (not MotionSaveActive->IsValid)
        return false;
    else
    {
        active = MotionSaveActive->Data[0];
        return true;
    }
}

/*! @brief Get current walk speed [cm/s, cm/s, rad/s]
    @param speed will be updated with the current commanded walk speed
    @return true if the data is valid
 */
bool NUSensorsData::getMotionWalkSpeed(vector<float>& speed)
{
    if (not MotionWalkSpeed->IsValid)
        return false;
    else
    {
        speed = MotionWalkSpeed->Data;
        return true;
    }
}

/*! @brief Gets the current walk maximum speed as [x cm/s, y cm/s yaw rad/s]
    @param speed will be updated with the current max walk speed
    @return true if the data is valid
 */
bool NUSensorsData::getMotionWalkMaxSpeed(vector<float>& speed)
{
    if (not MotionWalkMaxSpeed->IsValid)
        return false;
    else
    {
        speed = MotionWalkMaxSpeed->Data;
        return true;
    }
}

/*! @brief Get the current head completion time 
    @param time will be updated with the current head completion time
    @return true if the data is valid
 */
bool NUSensorsData::getMotionHeadCompletionTime(double& time)
{
    if (not MotionHeadCompletionTime->IsValid)
        return false;
    else
    {
        time = MotionHeadCompletionTime->Data[0];
        return true;
    }
}

/******************************************************************************************************************************************
                                                                                                                                Set Methods
 ******************************************************************************************************************************************/

/*! @brief Sets each of the static joint_id_t if the joint is in the list. Also sets id lists for accessing limbs. 
    @param joints a vector of strings where each string is a name of a joint
 */
void NUSensorsData::setAvailableJoints(const vector<string>& joints)
{
    m_joint_names = joints;
    // NOTE: This function is the same as setAvailableJoints in NUActionatorsData; so if you change this you probably want to change that too
    // first convert everything to lower case and remove whitespace and underscores
    vector<string> simplejointnames;
    string namebuffer, currentname, currentletter;
    for (unsigned int i=0; i<joints.size(); i++)
    {
        currentname = joints[i];
        // compare each letter to a space and an underscore
        for (unsigned int j=0; j<currentname.size(); j++)
        {
            currentletter = currentname.substr(j, 1);
            if (currentletter.compare(string(" ")) != 0 && currentletter.compare(string("_")) != 0)     // if it is neither then add the lower case version
                namebuffer += tolower(currentletter[0]);            
        }
        simplejointnames.push_back(namebuffer);
        namebuffer.clear();
    }
    
    for (unsigned int i=0; i<simplejointnames.size(); i++) 
    {
        if (simplejointnames[i].find("headyaw") != string::npos)
        {
            HeadYaw = i;
            m_head_ids.push_back(i);
        }
        else if (simplejointnames[i].find("headpitch") != string::npos)
        {
            HeadPitch = i;
            m_head_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lshoulderpitch") != string::npos)
        {
            LShoulderPitch = i;
            m_larm_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lshoulderroll") != string::npos)
        {
            LShoulderRoll = i;
            m_larm_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lelbowyaw") != string::npos)
        {
            LElbowYaw = i;
            m_larm_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lelbowroll") != string::npos)
        {
            LElbowRoll = i;
            m_larm_ids.push_back(i);
        }
        else if (simplejointnames[i].find("rshoulderpitch") != string::npos)
        {
            RShoulderPitch = i;
            m_rarm_ids.push_back(i);
        }
        else if (simplejointnames[i].find("rshoulderroll") != string::npos)
        {
            RShoulderRoll = i;
            m_rarm_ids.push_back(i);
        }
        else if (simplejointnames[i].find("relbowyaw") != string::npos)
        {
            RElbowYaw = i;
            m_rarm_ids.push_back(i);
        }
        else if (simplejointnames[i].find("relbowroll") != string::npos)
        {
            RElbowRoll = i;
            m_rarm_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lhipyawpitch") != string::npos)
        {
            LHipYawPitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lhipyaw") != string::npos)
        {
            LHipYaw = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lhippitch") != string::npos)
        {
            LHipPitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lhiproll") != string::npos)
        {
            LHipRoll = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lkneepitch") != string::npos)
        {
            LKneePitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lanklepitch") != string::npos)
        {
            LAnklePitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("lankleroll") != string::npos)
        {
            LAnkleRoll = i;
            m_lleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("rhipyawpitch") != string::npos)
        {
            RHipYawPitch = i;
            m_rleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("rhipyaw") != string::npos)
        {
            RHipYaw = i;
            m_rleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("rhippitch") != string::npos)
        {
            RHipPitch = i;
            m_rleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("rhiproll") != string::npos)
        {
            RHipRoll = i;
            m_rleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("rkneepitch") != string::npos)
        {
            RKneePitch = i;
            m_rleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("ranklepitch") != string::npos)
        {
            RAnklePitch = i;
            m_rleg_ids.push_back(i);
        }
        else if (simplejointnames[i].find("rankleroll") != string::npos)
        {
            RAnkleRoll = i;
            m_rleg_ids.push_back(i);
        }
    }
    // add the arms, torso and legs to the body_ids
    m_body_ids.insert(m_body_ids.end(), m_larm_ids.begin(), m_larm_ids.end());
    m_body_ids.insert(m_body_ids.end(), m_rarm_ids.begin(), m_rarm_ids.end());
    m_body_ids.insert(m_body_ids.end(), m_torso_ids.begin(), m_torso_ids.end());
    m_body_ids.insert(m_body_ids.end(), m_lleg_ids.begin(), m_lleg_ids.end());
    m_body_ids.insert(m_body_ids.end(), m_rleg_ids.begin(), m_rleg_ids.end());
    // add the head and the body_ids to the all_joint_ids
    m_all_joint_ids.insert(m_all_joint_ids.end(), m_head_ids.begin(), m_head_ids.end());
    m_all_joint_ids.insert(m_all_joint_ids.end(), m_body_ids.begin(), m_body_ids.end());
    
    // set total numbers in each limb
    m_num_head_joints = m_head_ids.size();
    m_num_arm_joints = m_larm_ids.size();
    m_num_torso_joints = m_torso_ids.size();
    m_num_leg_joints = m_lleg_ids.size();
    m_num_body_joints = m_body_ids.size();
    m_num_joints = m_all_joint_ids.size();
}

/*! @brief Sets the joint positions to the given values
    @param time the time the data was collected in milliseconds
    @param data the new joint position values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setJointPositions(double time, const vector<float>& data, bool iscalculated)
{
    setData(JointPositions, time, data, iscalculated);
}

/*! @brief Sets the joint velocities to the given values
     @param time the time the data was collected in milliseconds
     @param data the new joint velocities values
     @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setJointVelocities(double time, const vector<float>& data, bool iscalculated)
{
    setData(JointVelocities, time, data, iscalculated);
}

/*! @brief Sets the joint accelerations to the given values
    @param time the time the data was collected in milliseconds
    @param data the new joint accelerations values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setJointAccelerations(double time, const vector<float>& data, bool iscalculated)
{
    setData(JointAccelerations, time, data, iscalculated);
}

/*! @brief Sets the joint targets to the given values
    @param time the time the data was collected in milliseconds
    @param data the new joint targets values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setJointTargets(double time, const vector<float>& data, bool iscalculated)
{
    setData(JointTargets, time, data, iscalculated);
}

/*! @brief Sets the joint stiffnesses to the given values
    @param time the time the data was collected in milliseconds
    @param data the new joint stiffnesses values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setJointStiffnesses(double time, const vector<float>& data, bool iscalculated)
{
    setData(JointStiffnesses, time, data, iscalculated);
}

/*! @brief Sets the joint currents to the given values
    @param time the time the data was collected in milliseconds
    @param data the new joint currents values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setJointCurrents(double time, const vector<float>& data, bool iscalculated)
{
    setData(JointCurrents, time, data, iscalculated);
}

/*! @brief Sets the joint torques to the given values
    @param time the time the data was collected in milliseconds
    @param data the new joint torques values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setJointTorques(double time, const vector<float>& data, bool iscalculated)
{
    setData(JointTorques, time, data, iscalculated);
}

/*! @brief Sets the joint temperature to the given values
    @param time the time the data was collected in milliseconds
    @param data the new joint temperature values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setJointTemperatures(double time, const vector<float>& data, bool iscalculated)
{
    setData(JointTemperatures, time, data, iscalculated);
}

/*! @brief Sets the accelerometer to the given values
    @param time the time the data was collected in milliseconds
    @param data the new accelerometer values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setBalanceAccelerometer(double time, const vector<float>& data, bool iscalculated)
{
    setData(BalanceAccelerometer, time, data, iscalculated);
}

/*! @brief Sets the gyro to the given values
    @param time the time the data was collected in milliseconds
    @param data the gyro values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setBalanceGyro(double time, const vector<float>& data, bool iscalculated)
{
    setData(BalanceGyro, time, data, iscalculated);
}

/*! @brief Sets the left distance values to the given values
    @param time the time the data was collected in milliseconds
    @param data the new distance values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setDistanceLeftValues(double time, const vector<float>& data, bool iscalculated)
{
    setData(DistanceLeftValues, time, data, iscalculated);
}
/*! @brief Sets the right distance values to the given values
    @param time the time the data was collected in milliseconds
    @param data the new distance values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setDistanceRightValues(double time, const vector<float>& data, bool iscalculated)
{
    setData(DistanceRightValues, time, data, iscalculated);
}

/*! @brief Sets the foot sole values to the given values
 @param time the time the data was collected in milliseconds
 @param data the new foot sole values values
 @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setFootSoleValues(double time, const vector<float>& data, bool iscalculated)
{
    setData(FootSoleValues, time, data, iscalculated);
}

/*! @brief Sets the foot bumper to the given values
    @param time the time the data was collected in milliseconds
    @param data the new foot bumper values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setFootBumperValues(double time, const vector<float>& data, bool iscalculated)
{
    setData(FootBumperValues, time, data, iscalculated);
}

/*! @brief Sets the button values to the given values
    @param time the time the data was collected in milliseconds
    @param data the new button values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setButtonValues(double time, const vector<float>& data, bool iscalculated)
{
    setData(ButtonValues, time, data, iscalculated);
}

/*! @brief Sets the battery to the given values
    @param time the time the data was collected in milliseconds
    @param data the battery values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setBatteryValues(double time, const vector<float>& data, bool iscalculated)
{
    setData(BatteryValues, time, data, iscalculated);
}

/*! @brief Sets whether the fall engine is active
    @param time the timestamp
    @param active true if fall is active
 */ 
void NUSensorsData::setMotionFallActive(double time, bool active)
{
    vector<float> data(1,0);
    data[0] = active;
    setData(MotionFallActive, time, data, false);
}

/*! @brief Sets whether the getup engine is active
    @param time the timestamp
    @param active true if getup is active
 */ 
void NUSensorsData::setMotionGetupActive(double time, bool active)
{
    vector<float> data(1,0);
    data[0] = active;
    setData(MotionGetupActive, time, data, false);
}

/*! @brief Sets whether the kick engine is active
    @param time the timestamp
    @param active true if kick is active
 */ 
void NUSensorsData::setMotionKickActive(double time, bool active)
{
    vector<float> data(1,0);
    data[0] = active;
    setData(MotionKickActive, time, data, false);
}

/*! @brief Sets whether the save engine is active
    @param time the timestamp
    @param active true if save is active
 */ 
void NUSensorsData::setMotionSaveActive(double time, bool active)
{
    vector<float> data(1,0);
    data[0] = active;
    setData(MotionSaveActive, time, data, false);
}

/*! @brief Sets whether the script engine is active
    @param time the timestamp
    @param active true if script is active
 */
void NUSensorsData::setMotionScriptActive(double time, bool active)
{
    vector<float> data(1,0);
    data[0] = active;
    setData(MotionScriptActive, time, data, false);
}

/*! @brief Sets the current walk speed 
    @param time the timestamp
    @param speed [x,y,yaw]
 */
void NUSensorsData::setMotionWalkSpeed(double time, vector<float>& speed)
{
    setData(MotionWalkSpeed, time, speed, false);
}

/*! @brief Sets the current walk speed 
    @param time the timestamp
    @param speed the maximum speeds of [x,y,yaw]
 */
void NUSensorsData::setMotionWalkMaxSpeed(double time, vector<float>& speed)
{
    setData(MotionWalkMaxSpeed, time, speed, false);
}

/*! @brief Sets the completion time of the current head movement
    @param time the time the data was set in milliseconds
    @param completiontime the head movement completion time
 */
void NUSensorsData::setMotionHeadCompletionTime(double time, double completiontime)
{
    vector<float> ct(1,0);
    ct[0] = completiontime;
    setData(MotionHeadCompletionTime, time, ct, false);
}

/*! @brief Sets the GPS coordinates to the given values
    @param time the time the data was collected in milliseconds
    @param data the GPS values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setGPSValues(double time, const vector<float>& data, bool iscalculated)
{
    setData(GPS, time, data, iscalculated);
}

/*! @brief Sets the compass coordinates to the given values
    @param time the time the data was collected in milliseconds
    @param data the Compass values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setCompassValues(double time, const vector<float>& data, bool iscalculated)
{
    setData(Compass, time, data, iscalculated);
}

/* @brief Sets the data of the given sensor to the given data
   @param p_sensor a pointer to the sensor to be updated
   @param time the time the data was collected
   @param data the new data to be stored in p_sensor
   @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setData(sensor_t* p_sensor, double time, const vector<float>& data, bool iscalculated)
{
    CurrentTime = time;
    p_sensor->setData(time, data, iscalculated);
}

/******************************************************************************************************************************************
                                                                                                      Displaying Contents and Serialisation
 ******************************************************************************************************************************************/

/*! @brief Puts a user readable summary of the contents of the NUSensorsData class
    @param output the stream in which to put the summary
 */
void NUSensorsData::summaryTo(ostream& output) const
{
    for (unsigned int i=0; i<m_sensors.size(); i++)
        m_sensors[i]->summaryTo(output);
}

/*! @todo Implement this function
 */
void NUSensorsData::csvTo(ostream& output)
{
}

/*! @brief Returns the number of sensors in the NUSensorsData
 */
int NUSensorsData::size() const
{
    return m_sensors.size();
}

/*! @brief Put the entire contents of the NUSensorsData class into a stream
 */
ostream& operator<< (ostream& output, const NUSensorsData& p_data)
{
    output << p_data.size() << " ";
    for (int i=0; i<p_data.size(); i++)
        output << *p_data.m_sensors[i];
    return output;
}

/*! @brief Get the entire contents of the NUSensorsData class from a stream
 */
istream& operator>> (istream& input, NUSensorsData& p_data)
{
    p_data.m_sensors.clear();
    int numsensors;
    sensor_t insensor;
    sensor_t* sensor;
    input >> numsensors;
    double lastUpdateTime = 0;
    for (int i=0; i<numsensors; i++)
    {
        if(!input.good()) throw exception();
        input >> insensor;
        sensor = new sensor_t(insensor);
        p_data.m_sensors.push_back(sensor);
        p_data.updateNamedSensorPointer(sensor);
        if(sensor->Time > lastUpdateTime) lastUpdateTime = sensor->Time;
    }
    p_data.CurrentTime = lastUpdateTime;
    return input;
}

/*! @brief A helper function to update a named sensor pointer based on the id of p_sensor
    @param p_sensor the p_sensor->SensorID will be used to update one of the named pointers to point to p_sensor
 */
void NUSensorsData::updateNamedSensorPointer(sensor_t* p_sensor)
{
    switch (p_sensor->SensorID) 
    {
        case sensor_t::JOINT_POSITIONS:
            JointPositions = p_sensor;
            break;
        case sensor_t::JOINT_VELOCITIES:
            JointVelocities = p_sensor;
            break;
        case sensor_t::JOINT_ACCELERATIONS:
            JointAccelerations = p_sensor;
            break;
        case sensor_t::JOINT_TARGETS:
            JointTargets = p_sensor;
            break;
        case sensor_t::JOINT_STIFFNESSES:
            JointStiffnesses = p_sensor;
            break;
        case sensor_t::JOINT_CURRENTS:
            JointCurrents = p_sensor;
            break;
        case sensor_t::JOINT_TORQUES:
            JointTorques = p_sensor;
            break;
        case sensor_t::JOINT_TEMPERATURES:
            JointTemperatures = p_sensor;
            break;
        case sensor_t::KINEMATICS_LEFT_LEG_TRANSFORM:
            LeftLegTransform = p_sensor;
            break;
        case sensor_t::KINEMATICS_RIGHT_LEG_TRANSFORM:
            RightLegTransform = p_sensor;
            break;
        case sensor_t::KINEMATICS_SUPPORT_LEG_TRANSFORM:
            SupportLegTransform = p_sensor;
            break;
        case sensor_t::KINEMATICS_CAMERA_TRANSFORM:
            CameraTransform = p_sensor;
            break;
        case sensor_t::KINEMATICS_CAMERA_TO_GROUND_TRANSFORM:
            CameraToGroundTransform = p_sensor;
            break;
        case sensor_t::JOINT_ODOMETRY:
            Odometry = p_sensor;
            break;
        case sensor_t::JOINT_CAMERAHEIGHT:
            CameraHeight = p_sensor;
            break;
        case sensor_t::BALANCE_ACCELEROMETER:
            BalanceAccelerometer = p_sensor;
            break;
        case sensor_t::BALANCE_GYRO:
            BalanceGyro = p_sensor;
            break;
        case sensor_t::BALANCE_GYRO_OFFSET:
            BalanceGyroOffset = p_sensor;
            break;
        case sensor_t::BALANCE_ORIENTATION:
            BalanceOrientation = p_sensor;
            break;
        case sensor_t::BALANCE_HORIZON:
            BalanceHorizon = p_sensor;
            break;
        case sensor_t::BALANCE_ZMP:
            BalanceZMP = p_sensor;
            break;
        case sensor_t::BALANCE_FALLING:
            BalanceFalling = p_sensor;
            break;
        case sensor_t::BALANCE_FALLEN:
            BalanceFallen = p_sensor;
            break;
        case sensor_t::DISTANCE_LEFT_VALUES:
            DistanceLeftValues = p_sensor;
            break;
	case sensor_t::DISTANCE_RIGHT_VALUES:
            DistanceRightValues = p_sensor;
            break;
        case sensor_t::FOOT_SOLE_VALUES:
            FootSoleValues = p_sensor;
            break;
        case sensor_t::FOOT_BUMPER_VALUES:
            FootBumperValues = p_sensor;
            break;
        case sensor_t::FOOT_COP:
            FootCoP = p_sensor;
            break;
        case sensor_t::FOOT_FORCE:
            FootForce = p_sensor;
            break;
        case sensor_t::FOOT_CONTACT:
            FootContact = p_sensor;
            break;
        case sensor_t::FOOT_SUPPORT:
            FootSupport = p_sensor;
            break;
        case sensor_t::FOOT_IMPACT:
            FootImpact = p_sensor;
            break;
        case sensor_t::BUTTON_VALUES:
            ButtonValues = p_sensor;
            break;
        case sensor_t::BUTTON_TRIGGERS:
            ButtonTriggers = p_sensor;
            break;
        case sensor_t::BATTERY_VALUES:
            BatteryValues = p_sensor;
            break;
        case sensor_t::MOTION_FALL_ACTIVE:
            MotionFallActive = p_sensor;
            break;
        case sensor_t::MOTION_GETUP_ACTIVE:
            MotionGetupActive = p_sensor;
            break;
        case sensor_t::MOTION_KICK_ACTIVE:
            MotionKickActive = p_sensor;
            break;
        case sensor_t::MOTION_SAVE_ACTIVE:
            MotionSaveActive = p_sensor;
            break;
        case sensor_t::MOTION_SCRIPT_ACTIVE:
            MotionScriptActive = p_sensor;
            break;
        case sensor_t::MOTION_WALK_SPEED:
            MotionWalkSpeed = p_sensor;
            break;
        case sensor_t::MOTION_WALK_MAX_SPEED:
            MotionWalkMaxSpeed = p_sensor;
            break;            
        case sensor_t::MOTION_HEAD_COMPLETION_TIME:
            MotionHeadCompletionTime = p_sensor;
            break;
        case sensor_t::GPS_VALUES:
            GPS = p_sensor;
            break;
        default:
            break;
    }
}


