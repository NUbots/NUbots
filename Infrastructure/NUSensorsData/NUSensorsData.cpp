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

#include "Tools/Math/StlVector.h"
#include "debug.h"
#include "debugverbositynusensors.h"

#include <fstream>

int s_curr_id = NUData::NumCommonIds.Id+1;
vector<NUSensorsData::id_t*> NUSensorsData::m_ids;

// end effector sensors
const NUSensorsData::id_t NUSensorsData::LArmEndEffector(s_curr_id++, "LArmEndEffector", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::RArmEndEffector(s_curr_id++, "RArmEndEffector", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::LLegEndEffector(s_curr_id++, "LLegEndEffector", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::CameraTransform(s_curr_id++, "RLegEndEffector", NUSensorsData::m_ids);
// kinematic sensors
const NUSensorsData::id_t NUSensorsData::LLegTransform(s_curr_id++, "LLegTransform", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::RLegTransform(s_curr_id++, "RLegTransform", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::SupportLegTransform(s_curr_id++, "SupportLegTransform", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::CameraTransform(s_curr_id++, "CameraTransform", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::CameraToGroundTransform(s_curr_id++, "CameraToGroundTransform", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::CameraHeight(s_curr_id++, "CameraHeight", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::Odometry(s_curr_id++, "Odometry", NUSensorsData::m_ids);
// balance sensors
const NUSensorsData::id_t NUSensorsData::Accelerometer(s_curr_id++, "Accelerometer", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::Gyro(s_curr_id++, "Gyro", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::GyroOffset(s_curr_id++, "GyroOffset", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::Orientation(s_curr_id++, "Orientation", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::Horizon(s_curr_id++, "Horizon", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::Zmp(s_curr_id++, "Zmp", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::Falling(s_curr_id++, "Falling", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::Fallen(s_curr_id++, "Fallen", NUSensorsData::m_ids);
// touch sensors
const NUSensorsData::id_t NUSensorsData::LHandTouch(s_curr_id++, "LHandTouch", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::RHandTouch(s_curr_id++, "RHandTouch", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::LFootTouch(s_curr_id++, "LFootTouch", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::RFootTouch(s_curr_id++, "RFootTouch", NUSensorsData::m_ids);
// button sensors
const NUSensorsData::id_t NUSensorsData::MainButton(s_curr_id++, "MainButton", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::LeftButton(s_curr_id++, "LeftButton", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::RightButton(s_curr_id++, "RightButton", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::AllButton(s_curr_id++, "AllButton", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::AllButtonDurations(s_curr_id++, "AllButtonDurations", NUSensorsData::m_ids);
// distance sensors
const NUSensorsData::id_t NUSensorsData::LDistance(s_curr_id++, "LDistance", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::RDistance(s_curr_id++, "RDistance", NUSensorsData::m_ids);
// gps sensors
const NUSensorsData::id_t NUSensorsData::Gps(s_curr_id++, "Gps", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::Compass(s_curr_id++, "Compass", NUSensorsData::m_ids);
// battery sensors
const NUSensorsData::id_t NUSensorsData::BatteryVoltage(s_curr_id++, "BatteryVoltage", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::BatteryCurrent(s_curr_id++, "BatteryCurrent", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::BatteryCharge(s_curr_id++, "BatteryCharge", NUSensorsData::m_ids);
// motion sensors
const NUSensorsData::id_t NUSensorsData::MotionFallActive(s_curr_id++, "MotionFallActive", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::MotionGetupActive(s_curr_id++, "MotionGetupActive", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::MotionKickActive(s_curr_id++, "MotionKickActive", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::MotionSaveActive(s_curr_id++, "MotionSaveActive", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::MotionScriptActive(s_curr_id++, "MotionScriptActive", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::MotionWalkSpeed(s_curr_id++, "MotionWalkSpeed", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::MotionWalkMaxSpeed(s_curr_id++, "MotionWalkMaxSpeed", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::MotionHeadCompletionTime(s_curr_id++, "MotionHeadCompletionTime", NUSensorsData::m_ids);

/*! @brief Default constructor for NUSensorsData
 */
NUSensorsData::NUSensorsData()
{
#if DEBUG_NUSENSORS_VERBOSITY > 0
    debug << "NUSensorsData::NUSensorsData" << endl;
#endif
    
    CurrentTime = 0;
    
    m_ids.insert(m_ids.begin(), NUData::m_common_ids.begin(), NUData::m_common_ids.end());
    m_ids_copy = m_ids;
    m_id_to_indices = vector<vector<int> >(m_ids.size(), vector<int>());

    for (size_t i=0; i<m_ids.size(); i++)
        m_sensors.push_back(Sensor(m_ids[i]->Name));
}

NUSensorsData::~NUSensorsData()
{
    #if DEBUG_NUSENSORS_VERBOSITY > 0
        debug << "NUSensorsData::~NUSensorsData" << endl;
    #endif
}

void NUSensorsData::addSensors(const vector<string>& hardwarenames)
{
    // the model we use for sensors, is that every sensor is 'available', but the data may be invalid.
    for (size_t i=NumCommonIds.Id; i<m_ids.size(); i++)
        m_id_to_indices[i].push_back(i);
    
    // the addDevices function will just setup the groups for the joints
    addDevices(hardwarenames);
}

/******************************************************************************************************************************************
                                                                                                                                Get Methods
 ******************************************************************************************************************************************/

/*! @brief Gets the joint position
    @param id the id of the joint
    @param data will be updated with the angle
    @return true if valid, false if invalid
 */
bool NUSensorsData::getPosition(const id_t id, float& data)
{
    return getJointData(id, PositionId, data);
}

/*! @brief Gets the joint positions
    @param id the id of the group of joints
    @param data will be updated with the joint angles
    @return true if valid, false if invalid
 */
bool NUSensorsData::getPosition(const id_t id, vector<float>& data)
{
    return getJointData(id, PositionId, data);
}

/*! @brief Gets the joint velocity
    @param id the id of the joint
    @param data will be updated with joint velocity
    @return true if valid, false if invalid
 */
bool NUSensorsData::getVelocity(const id_t id, float& data)
{
    return getJointData(id, VelocityId, data);
}

/*! @brief Gets the joint velocities
    @param id the id of the group of joints
    @param data will be updated with joint velocities
    @return true if valid, false if invalid
 */
bool NUSensorsData::getVelocity(const id_t id, vector<float>& data)
{
    return getJointData(id, VelocityId, data);
}

/*! @brief Gets the joint acceleration
    @param id the id of the joint
    @param data will be updated with joint acceleration
    @return true if valid, false if invalid
 */
bool NUSensorsData::getAcceleration(const id_t id, float& data)
{
    return getJointData(id, AccelerationId, data);
}

/*! @brief Gets the joint accelerations
    @param id the id of the group of joints
    @param data will be updated with joint accelerations
    @return true if valid, false if invalid
 */
bool NUSensorsData::getAcceleration(const id_t id, vector<float>& data)
{
    return getJointData(id, AccelerationId, data);
}

/*! @brief Gets the joint target
    @param id the id of the joint
    @param data will be updated with joint target
    @return true if valid, false if invalid
 */
bool NUSensorsData::getTarget(const id_t id, float& data)
{
    return getJointData(id, TargetId, data);
}

/*! @brief Gets the joint targets
    @param id the id of the group of joints
    @param data will be updated with joint targets
    @return true if valid, false if invalid
 */
bool NUSensorsData::getTarget(const id_t id, vector<float>& data)
{
    return getJointData(id, TargetId, data);
}

/*! @brief Gets the joint stiffness
    @param id the id of the joint
    @param data will be updated with joint stiffness
    @return true if valid, false if invalid
 */
bool NUSensorsData::getStiffness(const id_t id, float& data)
{
    return getJointData(id, StiffnessId, data);
}

/*! @brief Gets the joint stiffnesses
    @param id the id of the group of joints
    @param data will be updated with joint stiffnesses
    @return true if valid, false if invalid
 */
bool NUSensorsData::getStiffness(const id_t id, vector<float>& data)
{
    return getJointData(id, StiffnessId, data);
}

/*! @brief Gets the joint current
    @param id the id of the joint
    @param data will be updated with joint current
    @return true if valid, false if invalid
 */
bool NUSensorsData::getCurrent(const id_t id, float& data)
{
    return getJointData(id, CurrentId, data);
}

/*! @brief Gets the joint currents
    @param id the id of the group of joints
    @param data will be updated with joint currents
    @return true if valid, false if invalid
 */
bool NUSensorsData::getCurrent(const id_t id, vector<float>& data)
{
    return getJointData(id, CurrentId, data);
}

/*! @brief Gets the joint torque
    @param id the id of the joint
    @param data will be updated with joint torque
    @return true if valid, false if invalid
 */
bool NUSensorsData::getTorque(const id_t id, float& data)
{
    return getJointData(id, TorqueId, data);
}

/*! @brief Gets the joint torques
    @param id the id of the group of joints
    @param data will be updated with joint torques
    @return true if valid, false if invalid
 */
bool NUSensorsData::getTorque(const id_t id, vector<float>& data)
{
    return getJointData(id, TorqueId, data);
}

/*! @brief Gets the joint temperature
    @param id the id of the joint
    @param data will be updated with joint temperature
    @return true if valid, false if invalid
 */
bool NUSensorsData::getTemperature(const id_t id, float& data)
{
    return getJointData(id, TemperatureId, data);
}

/*! @brief Gets the joint temperatures
    @param id the id of the group of joints
    @param data will be updated with joint temperatures
    @return true if valid, false if invalid
 */
bool NUSensorsData::getTemperature(const id_t id, vector<float>& data)
{
    return getJointData(id, TemperatureId, data);
}

/*! @brief Gets the position of an end effector
    @param id the id of the end effector
    @param data will be updated with the position
    @return true if valid, false if invalid
 */
bool NUSensorsData::getEndPosition(const id_t id, vector<float>& data)
{
    if (id == LArm or id == LHand or id == LArmEndEffector)
        return false;
    else if (id == RArm or id == RHand or id == RArmEndEffector)
        return false;
    else if (id == LLeg or id == LLeg or id == LLegEndEffector)
    {
        vector<float>& leftlegtransform;
        bool successful = get(LLegTransform, leftlegtransform);
        if (successful
    }
}

bool NUSensorsData::getCameraHeight(float& data)
{
    return get(CameraHeight, data);
}

bool NUSensorsData::getHorizon(vector<float>& data)
{
    return get(Horizon, data);
}

bool NUSensorsData::getOdometry(vector<float>& data)
{
    bool successful = get(Odometry, data);
    if (successful)
        set(Odometry, vector<float> (data.size(), 0));
    return successful;
}

bool NUSensorsData::getAccelerometer(vector<float>& data)
{
    get(Accelerometer, data);
}

bool NUSensorsData::getGyro(vector<float>& data)
{
    vector<float> gyro, offset; 
    bool successful = get(Gyro, gyro);
    successful &= get(GyroOffset, offset);
    if (successful)
        data = gyro - offset;
    return successful;
}

bool NUSensorsData::getOrientation(vector<float>& data)
{
    return get(Orientation, data);
}

bool NUSensorsData::getFalling(vector<float>& data)
{
    return get(Falling, data);
}

bool NUSensorsData::getFallen(vector<float>& data)
{
    return get(Fallen, data);
}

bool NUSensorsData::getBumper(const id_t& id, float& data)
{
    
    
    
}

bool NUSensorsData::getForce(const id_t& id, float& data)
{
}

bool NUSensorsData::getContact(const id_t& id, float& data)
{
}

bool NUSensorsData::getSupport(const id_t& id, float& data)
{
}

bool NUSensorsData::getImpact(const id_t& id, float& data)
{
    
}

bool NUSensorsData::getZmp(const id_t& id, vector<float>& data)
{
}

bool NUSensorsData::getCoP(const id_t& id, vector<float>& data)
{
}

bool NUSensorsData::getButton(const id_t& id, float& data)
{
}

bool NUSensorsData::getButton(const id_t& id, vector<float>& data)
{
}

bool NUSensorsData::getButtonDuration(const id_t& id, float& data)
{
}

bool NUSensorsData::getButtonDuration(const id_t& id, vector<float>& data)
{
}

bool NUSensorsData::getDistance(const id_t& id, float& data)
{
}

bool NUSensorsData::getDistance(const id_t& id, vector<float>& data)
{
}

bool NUSensorsData::getGps(vector<float>& data)
{
}

bool NUSensorsData::getCompass(vector<float>& data)
{
}

bool NUSensorsData::getBatteryVoltage(float& data)
{
}

bool NUSensorsData::getBatteryCurrent(float& data)
{
}

bool NUSensorsData::getBatteryCharge(float& data)
{
}

/*! @brief Gets boolean sensor reading for id
    @param id the id of the sensor
    @param data will be updated with the sensor reading
    @return true if the data is valid, false otherwise
 */
bool NUSensorsData::get(const id_t& id, bool& data)
{
    vector<int>& ids = mapIdToIndices(id);
    float floatBuffer;
    if (ids.size() == 1)
    {
        bool successful = m_sensors[ids[0]].get(floatBuffer);
        data = static_cast<bool>(floatBuffer);
        return successful;
    }
    else
        return false;
}

/*! @brief Gets float sensor reading for id
    @param id the id of the sensor
    @param data will be updated with the sensor reading
    @return true if the data is valid, false otherwise
 */
bool NUSensorsData::get(const id_t& id, float& data)
{
    vector<int>& ids = mapIdToIndices(id);
    if (ids.size() == 1)
        return m_sensors[ids[0]].get(data);
    else
        return false;
}

/*! @brief Gets double sensor reading for id
    @param id the id of the sensor
    @param data will be updated with the sensor reading
    @return true if the data is valid, false otherwise
 */
bool NUSensorsData::get(const id_t& id, double& data)
{
    float floatBuffer;
    bool successful = get(id, floatBuffer);
    data = static_cast<double>(floatBuffer);
    return successful;
}

/*! @brief Gets vector of sensor readings for id. If id is a group then each element in data will be a float from each member of the group.
    @param id the id of the sensor(s)
    @param data will be updated with the sensor reading(s)
    @return true if the data is valid, false otherwise
 */
bool NUSensorsData::get(const id_t& id, vector<float>& data)
{
    vector<int>& ids = mapIdToIndices(id);
    size_t numids = ids.size();
    if (numids == 0)
        return false;
    else if (numids == 1)
        return m_sensors[ids[0]].get(data);
    else
    {
        data.clear();
        data.reserve(numids);
        bool successful = true;
        float floatBuffer;
        for (size_t i=0; i<ids.size(); i++)
        {
            successful &= m_sensors[ids[i]].get(floatBuffer);
            data.push_back(floatBuffer);
        }
        return successful;
    }
}

/*! @brief Gets matrix of sensor readings for id. If id is a group then each element in data will be a vector from each member of the group.
    @param id the id of the sensor(s)
    @param data will be updated with the sensor reading(s)
    @return true if the data is valid, false otherwise
 */
bool NUSensorsData::get(const id_t& id, vector<vector<float> >& data)
{
    vector<int>& ids = mapIdToIndices(id);
    size_t numids = ids.size();
    if (numids == 0)
        return false;
    else if (numids == 1)
        return m_sensors[ids[0]].get(data);
    else
    {
        data.clear();
        data.reserve(numids);
        bool successful = true;
        vector<float> vectorBuffer;
        for (size_t i=0; i<ids.size(); i++)
        {
            successful &= m_sensors[ids[i]].get(vectorBuffer);
            data.push_back(vectorBuffer);
        }
        return successful;
    }
}

/*! @brief Gets string sensor reading for id
    @param id the id of the sensor
    @param data will be updated with the sensor reading
    @return true if the data is valid, false otherwise
 */
bool NUSensorsData::get(const id_t& id, string& data)
{
    vector<int>& ids = mapIdToIndices(id);
    if (ids.size() == 1)
        return m_sensors[ids[0]].get(data);
    else
        return false;
}

/* Gets a single type of joint sensor information, eg. a Temperature.
   @param id the id of the group of joints
   @param in the index into a joint sensor vector for the desired type of information
   @param data will be updated the the sensor readings
   @return true if the data is valid, false otherwise
 */
bool NUSensorsData::getJointData(const id_t& id, const JointSensorIndices& in, float& data)
{
    vector<int>& ids = mapIdToIndices(id);
    if (ids.size() == 1)
    {
        vector<float> vectorBuffer;
        if (m_sensors[ids[0]].get(vectorBuffer))
        {
            data = vectorBuffer[in];
            if (isnan(data))
                return false;
            else
                return true;
        }
        else
            return false;
    }
    else
        return false;
}

/* Gets a vector of a single type of joint sensor information, eg. a vector of Temperatures.
   @param id the id of the group of joints
   @param in the index into a joint sensor vector for the desired type of information
   @param data will be updated the the sensor readings
   @return true if the data is valid, false otherwise
 */
bool NUSensorsData::getJointData(const id_t& id, const JointSensorIndices& in, vector<float>& data)
{
    vector<int>& ids = mapIdToIndices(id);
    size_t numids = ids.size();
    if (numids <= 1)
        return false;
    else
    {
        data.clear();
        data.reserve(numids);
        bool successful = true;
        vector<float> vectorBuffer;
        float floatBuffer;
        for (size_t i=0; i<numids; i++)
        {
            successful &= m_sensors[ids[i]].get(vectorBuffer);
            floatBuffer = vectorBuffer[in];
            successful & = not isnan(floatBuffer);
            data.push_back(floatBuffer);
        }
        return successful;
    }
}

/* Gets a single type of end effector information, eg. a bumper value
   @param id the id of the end effector
   @param in the index into a end effector sensor vector for the desired type of information
   @param data will be updated the the sensor readings
   @return true if the data is valid, false otherwise
 */
bool NUSensorsData::getEndEffectorData(const id_t& id, const EndEffectorIndices& in, float& data)
{
    // map similar ids to the proper EndEffector ids
    id_t& e_id = id;
    if (e_id == LArm or e_id == LHand)
        e_id = LArmEndEffector;
    else if (e_id == RArm or e_id == RHand)
        e_id = RArmEndEffector;
    else if (e_id == LLeg or e_id == LFoot)
        e_id = LLegEndEffector;
    else if (e_id == RLeg or e_id == RFoot)
        e_id = RLegEndEffector;
    
    // proceed as usual with the proper end effector id
    vector<int>& ids = mapIdToIndices(e_id);
    if (ids.size() == 1)
    {
        vector<float> vectorBuffer;
        if (m_sensors[ids[0]].get(vectorBuffer))
        {
            data = vectorBuffer[in];
            if (isnan(data))
                return false;
            else
                return true;
        }
        else
            return false;
    }
    else
        return false;
}

/* @brief Gets the transform matrix of the left leg
   @param value will be updated with the left leg transform
 */
bool NUSensorsData::getLeftLegTransform(Matrix& value)
{
    return false;
    /*
    if (LeftLegTransform == NULL || LeftLegTransform->IsValid == false)
        return false;
    else
    {
        value = Matrix4x4fromVector(LeftLegTransform->Data);
        return true;
    }*/
}

/* @brief Gets the transform matrix of the right leg
   @param value will be updated with the right leg transform
 */
bool NUSensorsData::getRightLegTransform(Matrix& value)
{
    return false;
    /*
    if (RightLegTransform == NULL || RightLegTransform->IsValid == false)
        return false;
    else
    {
        value = Matrix4x4fromVector(RightLegTransform->Data);
        return true;
    }
     */
}

/* @brief Gets the transform matrix of the support leg
   @param value will be updated with the support leg transform
 */
bool NUSensorsData::getSupportLegTransform(Matrix& value)
{
    return false;
    /*
    if (SupportLegTransform == NULL || SupportLegTransform->IsValid == false)
        return false;
    else
    {
        value = Matrix4x4fromVector(SupportLegTransform->Data);
        return true;
    }*/
}

/* @brief Gets the transform matrix of the camera
   @param value will be updated with the camera transform
 */
bool NUSensorsData::getCameraTransform(Matrix& value)
{
    return false;
    /*
    if (CameraTransform == NULL || CameraTransform->IsValid == false)
        return false;
    else
    {
        value = Matrix4x4fromVector(CameraTransform->Data);
        return true;
    }*/
}

/* @brief Gets the transform matrix converting from the camera coordinates to ground based coordinates
   @param value will be updated with the camera to ground transform
 */
bool NUSensorsData::getCameraToGroundTransform(Matrix& value)
{
    return false;
    /*if (CameraToGroundTransform == NULL || CameraToGroundTransform->IsValid == false)
        return false;
    else
    {
        value = Matrix4x4fromVector(CameraToGroundTransform->Data);
        return true;
    }*/
}

bool NUSensorsData::getOdometryData(vector<float>& values)
{
    return false;
}

/******************************************************************************************************************************************
                                                                                                                 Convienent sub-get Methods
 ******************************************************************************************************************************************/

/*! @brief Returns true if the robot is falling over, false if it is not falling (or it is impossble to tell)
 */
bool NUSensorsData::isFalling()
{
    return false;
    /*
    if (BalanceFalling == NULL || BalanceFalling->IsValid == false)       // if there is no balance sensor it is impossible to tell it is falling over
        return false;       
    else if (BalanceFalling->Data[0] > 0)
        return true;
    else
        return false;*/
}

/*! @brief Returns true if the robot has fallen over, false if it hasn't (or it is impossible to tell)
 */
bool NUSensorsData::isFallen()
{
    return false;
    /*
    if (BalanceFallen == NULL || BalanceFallen->IsValid == false)       // if there is no balance sensor it is impossible to tell it has fallen over
        return false;       
    else if (BalanceFallen->Data[0] > 0)
        return true;
    else
        return false;*/
}

/*! @brief Returns true if the robot is on the ground, false otherwise
 */
bool NUSensorsData::isOnGround()
{
    return false;
    /*
    if (FootContact == NULL || FootContact->IsValid == false)
        return true;
    else if (FootContact->Data[2] > 0)
        return true;
    else
        return false;*/
}

/*! @brief Returns true if the robot is currently incapacitated. A robot is incapacitated if it is falling, fallen, not on the ground or getting up
 */
bool NUSensorsData::isIncapacitated()
{
    return false;
    /*
    bool gettingup = false;
    getMotionGetupActive(gettingup);
    return isFalling() or isFallen() or not isOnGround() or gettingup;*/
}

/*! @brief Returns true has impacted in the ground in this cycle
    @param footid the foot you want to know about
    @param time time will be updated with the time at which the last impact on that foot occured.
    @return true if the foot hit the ground in *this* cycle, it will be false otherwise (ie it will be false the cycle after the impact; that is what the time is for ;))
 */
bool NUSensorsData::footImpact(id_t footid, float& time)
{
    return false;
    /*
    if (FootImpact == NULL || FootImpact->IsValid == false)
    {
        time = 0;
        return false;
    }
    else if (footid == LLeg)
        time = FootImpact->Data[0];
    else if (footid == RLeg)
        time = FootImpact->Data[1];
    else if (footid == All)
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
     */
}

/******************************************************************************************************************************************
                                                                                                                                Set Methods
 ******************************************************************************************************************************************/

/*! @brief Sets the current sensor reading for id
    @param id the id of the targetted sensor
    @param time the time in ms the value was captured
    @param data the data
 */
void NUSensorsData::set(const id_t& id, double time, bool data)
{
    set(id, time, static_cast<float>(data));
}

/*! @brief Sets the current sensor reading for id
    @param id the id of the targetted sensor
    @param time the time in ms the value was captured
    @param data the data
 */
void NUSensorsData::set(const id_t& id, double time, const float& data)
{
    #if DEBUG_NUSENSORS_VERBOSITY > 4
        debug << "NUSensorsData::set(" << id.Name << "," << time << "," << data << ")" << endl;
    #endif
    vector<int>& ids = mapIdToIndices(id);
    for (size_t i=0; i<ids.size(); i++)
        m_sensors[ids[i]].set(time, data);
}

/*! @brief Sets the current sensor reading for id. If id is a group the each element of data will be given to each member of the group
    @param id the id of the targetted sensor
    @param time the time in ms the value was captured
    @param data the data
 */
void NUSensorsData::set(const id_t& id, double time, const vector<float>& data)
{
    #if DEBUG_NUSENSORS_VERBOSITY > 4
        debug << "NUSensorsData::set(" << id.Name << "," << time << "," << data << ")" << endl;
    #endif
    vector<int>& ids = mapIdToIndices(id);
    size_t numids = ids.size();
    if (numids == 0)
        return;
    else if (numids == 1)
    {   // if id is a single sensor
        m_sensors[ids[0]].set(time, data);
    }
    else if (numids == data.size())
    {   // if id is a group of sensors
        for (size_t i=0; i<numids; i++)
            m_sensors[ids[i]].set(time, data[i]);
    }
    else
    {
        debug << "NUSensors::set(" << id.Name << "," << time << "," << data << "). The data is incorrectly formatted. ";
        debug << "data.size():" << data.size() << " must be ids.size():" << numids << endl;
    }
}

/*! @brief Sets the current sensor reading for id. If id is a group the each element of data will be given to each member of the group
    @param id the id of the targetted sensor
    @param time the time in ms the value was captured
    @param data the data
 */
void NUSensorsData::set(const id_t& id, double time, const vector<vector<float> >& data)
{
    #if DEBUG_NUSENSORS_VERBOSITY > 4
        debug << "NUSensorsData::set(" << id.Name << "," << time << "," << data << ")" << endl;
    #endif
    vector<int>& ids = mapIdToIndices(id);
    size_t numids = ids.size();
    if (numids == 0)
        return;
    else if (numids == 1)
    {   // if id is a single sensor
        m_sensors[ids[0]].set(time, data);
    }
    else if (numids == data.size())
    {   // if id is a group of sensors
        for (size_t i=0; i<numids; i++)
            m_sensors[ids[i]].set(time, data[i]);
    }
    else
    {
        debug << "NUSensors::set(" << id.Name << "," << time << "," << data << "). The data is incorrectly formatted. ";
        debug << "data.size():" << data.size() << " must be ids.size():" << numids << endl;
    }
}

/*! @brief Sets the current sensor reading for id
    @param id the id of the targetted sensor
    @param time the time in ms the value was captured
    @param data the data
 */
void NUSensorsData::set(const id_t& id, double time, const string& data)
{
    #if DEBUG_NUSENSORS_VERBOSITY > 4
        debug << "NUSensorsData::set(" << id.Name << "," << time << ",\"" << data << "\")" << endl;
    #endif
    vector<int>& ids = mapIdToIndices(id);
    for (size_t i=0; i<ids.size(); i++)
        m_sensors[ids[i]].set(time, data);
}

/*! @brief Sets the readings for sensor id to be invalid 
    @param id the id of the targetted sensor
 */
void NUSensorsData::setAsInvalid(const id_t& id)
{
    vector<int>& ids = mapIdToIndices(id);
    for (size_t i=0; i<ids.size(); i++)
        m_sensors[ids[i]].setAsInvalid();
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
        m_sensors[i].summaryTo(output);
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
    /*output << p_data.size() << " ";
    for (int i=0; i<p_data.size(); i++)
        output << *p_data.m_sensors[i];
    return output;*/
}

/*! @brief Get the entire contents of the NUSensorsData class from a stream
 */
istream& operator>> (istream& input, NUSensorsData& p_data)
{
    /*p_data.m_sensors.clear();
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
    return input;*/
}


