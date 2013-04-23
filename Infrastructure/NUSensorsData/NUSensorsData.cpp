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
#include <limits>

int s_curr_id = NUData::m_num_common_ids+1; 
vector<NUSensorsData::id_t*> NUSensorsData::m_ids;

// end effector sensors
const NUSensorsData::id_t NUSensorsData::LArmEndEffector(s_curr_id++, "LArmEndEffector", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::RArmEndEffector(s_curr_id++, "RArmEndEffector", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::LLegEndEffector(s_curr_id++, "LLegEndEffector", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::RLegEndEffector(s_curr_id++, "RLegEndEffector", NUSensorsData::m_ids);
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
const NUSensorsData::id_t NUSensorsData::OrientationHardware(s_curr_id++, "OrientationHardware", NUSensorsData::m_ids);
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
// distance sensors
const NUSensorsData::id_t NUSensorsData::LDistance(s_curr_id++, "LDistance", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::RDistance(s_curr_id++, "RDistance", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::LaserDistance(s_curr_id++, "LaserDistance", NUSensorsData::m_ids);
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

const unsigned int NUSensorsData::m_num_sensor_ids = s_curr_id;

/*! @brief Default constructor for NUSensorsData
 */
NUSensorsData::NUSensorsData() : NUData(), TimestampedData()
{
#if DEBUG_NUSENSORS_VERBOSITY > 0
    debug << "NUSensorsData::NUSensorsData" << endl;
#endif
    CurrentTime = 0;
    PreviousTime = 0;

    // If this has already been initialised, don't do it again or bad stuff will happen.
    if(m_ids.size() < m_num_sensor_ids)
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
                                                                                                          Get Methods For Joint Information
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

/******************************************************************************************************************************************
                                                                                                   Get Methods For End Effector Information
 ******************************************************************************************************************************************/

/*! @brief Gets the bumper value for the end effector
    @param id the id of the end effector
    @param data will be updated with bumper value
    @return true if valid, false if invalid
 */
bool NUSensorsData::getBumper(const id_t& id, float& data)
{
    return getEndEffectorData(id, BumperId, data);
}

/*! @brief Gets the total force on the end effector
    @param id the id of the end effector
    @param data will be updated with force value
    @return true if valid, false if invalid
 */
bool NUSensorsData::getForce(const id_t& id, float& data)
{
    return getEndEffectorData(id, ForceId, data);
}

/*! @brief Gets the contact (touching something or not) value for the end effector
    @param id the id of the end effector
    @param data will be updated with contact value (0 means no contact, 1 means there is contact (with the ground))
    @return true if valid, false if invalid
 */
bool NUSensorsData::getContact(const id_t& id, bool& data)
{
    float buffer;
    bool successful = getEndEffectorData(id, ContactId, buffer);
    data = static_cast<bool>(buffer);
    return successful;
}

/*! @brief Gets the support (supporting the robot) value for the end effector
    @param id the id of the end effector
    @param data will be updated with support value (0 means this end effector is not supporting the robot, 1 means it is)
    @return true if valid, false if invalid
 */
bool NUSensorsData::getSupport(const id_t& id, bool& data)
{
    float buffer;
    bool successful = getEndEffectorData(id, SupportId, buffer);
    data = static_cast<bool>(buffer);
    return successful;
}

/*! @brief Gets the centre of pressure for the end effector
    @param id the id of the end effector
    @param data will be updated with centre of presssure [x(cm), y(cm)]
    @return true if valid, false if invalid
 */
bool NUSensorsData::getCoP(const id_t& id, vector<float>& data)
{
    data = vector<float>(2);
    bool successful = true;
    successful &= getEndEffectorData(id, CoPXId, data[0]);
    successful &= getEndEffectorData(id, CoPYId, data[1]);
    return successful;
}


/*! @brief Gets the position of an end effector. For example to get the position of the left leg use getEndPosition(NUSensorsData::LLeg, data)
    @param id the id of the end effector
    @param data will be updated with the position [x, y, z, roll, pitch, yaw]
    @return true if valid, false if invalid
 */
bool NUSensorsData::getEndPosition(const id_t id, vector<float>& data)
{
    data = vector<float>(6);
    bool successful = true;
    successful &= getEndEffectorData(id, EndPositionXId, data[0]);
    successful &= getEndEffectorData(id, EndPositionYId, data[1]);
    successful &= getEndEffectorData(id, EndPositionZId, data[2]);
    successful &= getEndEffectorData(id, EndPositionRollId, data[3]);
    successful &= getEndEffectorData(id, EndPositionPitchId, data[4]);
    successful &= getEndEffectorData(id, EndPositionYawId, data[5]);
    return successful;
}

/******************************************************************************************************************************************
                                                                                                      Get Methods For Kinematic Information
 ******************************************************************************************************************************************/

/*! @brief Get the camera height in cm 
 	@param data will be update with the height in cm
 	@return true if valid, false if invalid
 */
bool NUSensorsData::getCameraHeight(float& data)
{
    return get(CameraHeight, data);
}

/*! @brief Gets the horizon line [A, B, C] where Ax + By = C is the equation of the horizon line
 	@param data will be updated with the [A, B, C] of the line
 	@return true if valid, false if invalid
 */
bool NUSensorsData::getHorizon(vector<float>& data)
{
    return get(Horizon, data);
}

/*! @brief Gets the current odometry data [delta_x (cm), delta_y (cm), delta_yaw (rad)]
 	@param data will be updated with the current odometry data [delta_x (cm), delta_y (cm), delta_yaw (rad)] since the last call to getOdometryData
 	@return true if valid, false if invalid
 */
bool NUSensorsData::getOdometry(vector<float>& data)
{
    const static vector<float> zeros(3,0);
    if (get(Odometry, data))
    {
        set(Odometry, CurrentTime, zeros);			// reset the current odometry value
        return true;
    }
    else
        return false;
}

/******************************************************************************************************************************************
                                                                                                        Get Methods For Balance Information
 ******************************************************************************************************************************************/

/*! @brief Get the accelerometer data [x(cm/s/s), y(cm/s/s), z(cm/s/s)] 
    @param data will be update with the [x(cm/s/s), y(cm/s/s), z(cm/s/s)] 
    @return true if valid, false if invalid
 */
bool NUSensorsData::getAccelerometer(vector<float>& data)
{
    return get(Accelerometer, data);
}

/*! @brief Get the gyro data [roll(rad/s), pitch(rad/s), yaw(rad/s)] after the gyro offset has been applied
    @param data will be update with the [roll(rad/s), pitch(rad/s), yaw(rad/s)]
    @return true if valid, false if invalid
 */
bool NUSensorsData::getGyro(vector<float>& data)
{
    vector<float> gyro, offset; 
    bool successful = get(Gyro, gyro);
    successful &= get(GyroOffset, offset);
    if (successful)
        data = gyro - offset;
    return successful;
}

/*! @brief Gets the orientation of the torso relative to the gravity vector [x(rad), y(rad), z(rad)]
 	@param data will be updated with the [x(rad), y(rad), z(rad)]
 	@return true is valid, false if invalid
 */
bool NUSensorsData::getOrientation(vector<float>& data)
{
    return get(Orientation, data);
}

/*! @brief Gets the falling vector [total, left, right, forward, backward] where each will be 0 if not falling, and 1 if falling in that direction
 	@param data will be updated with the [total, left, right, forward, backward]
 	@return true if valid, false if invalid
 */
bool NUSensorsData::getFalling(vector<float>& data)
{
    return get(Falling, data);
}

/*! @brief Gets the fallen vector [total, left, right, forward, backward] where each will be 0 if not fallen, and 1 if fallen in that direction
 	@param data will be updated with the [total, left, right, forward, backward]
 	@return true if valid, false if invalid
 */
bool NUSensorsData::getFallen(vector<float>& data)
{
    return get(Fallen, data);
}

/*! @brief Gets the zmp [x(cm), y(cm)]
 	@param id the id of the zmp you want (All/Body, LFoot, RFoot)
 	@param data will be updated with the [x(cm), y(cm)]
 	@return true if valid, false if invalid
 */
bool NUSensorsData::getZmp(const id_t& id, vector<float>& data)
{
    (void)(id); // To stop compiler warnings.
    // TODO: this isn't right, it will always return the 'all' zmp.
    return get(Zmp, data);
}

/******************************************************************************************************************************************
                                                                                                  Get Methods For Other Sensors Information
 ******************************************************************************************************************************************/

/*! @brief Gets the Gps data [x(cm), y(cm), z(cm)] relative to the centre of the 'field'
 	@param data will be updated with the [x(cm), y(cm), z(cm)]
 	@return true if valid, false if invalid
 */
bool NUSensorsData::getGps(vector<float>& data)
{
    return get(Gps, data);
}

/*! @brief Gets the Compass value 0 being 'north' or towards the yellow goal in radians
    @param data will be updated with the bearing
    @return true if valid, false if invalid
 */
bool NUSensorsData::getCompass(float& data)
{
    return get(Compass, data);
}

/*! @brief Gets the distance data from the selected sensor.
 
 
    Note. The description below is how this SHOULD work, not how it actually is at the moment.
    For now its just a flat list of distances, that is there is no angle information returned.
 
 	For an ultrasonic sensor; the data will be formatted as follows:
 		[[angle_min, angle_max], [echo0,echo1,echo2,....]] where angle_min and angle_max specify the size of the ulrasonic's detection cone
 
 	For a 2d laser scanner or infrared array the data will be formatted as follows:
 		[[angle0,distance0],[angle1,distance1], ... , [angleN,distanceN]] where there are N sensors in the array
 
 	For a 3d laser scanner or infrared array the data will be formatted as follows:
 		[[angleX0,angleY0,distance0],[angleX1,angleY1,distance1], ... , [angleXN,angleYN,distanceN]] where there are N sensors in the array
 
 	@param id the id of the distance sensor you want
 	@param data will be updated the distances
 	@return true if the data is valid, false if not
 */
bool NUSensorsData::getDistance(const id_t& id, vector<float>& data)
{
    if (id < LDistance or id > LaserDistance)		// check the id is for a distance sensor
        return false;
	return get(id, data);
}

/*! @brief Gets the button value (0 when not pressed, 1 when pressed)
    @param id the id of the button you want
    @param data will be updated with the value
    @return true if valid, false if invalid
 */
bool NUSensorsData::getButton(const id_t& id, float& data)
{
    return getButtonData(id, StateId, data);
}

/*! @brief Gets the duration the button was last depressed for in ms
 	@param id the id of the button you want
 	@param data will be updated with the duration
 	@return true if valid, false if invalid
 */
bool NUSensorsData::getButtonDuration(const id_t& id, float& data)
{
    return getButtonData(id, DurationId, data);
}

/*! @brief Gets the battery voltage in Volts
 	@param data will be updated with the voltage
 	@return true is valid, false if invalid
 */
bool NUSensorsData::getBatteryVoltage(float& data)
{
    return get(BatteryVoltage, data);
}

/*! @brief Gets the battery current in Amperes
    @param data will be updated with the current
    @return true is valid, false if invalid
 */
bool NUSensorsData::getBatteryCurrent(float& data)
{
    return get(BatteryCurrent, data);
}

/*! @brief Gets the battery charge in %
    @param data will be updated with the percent charged
    @return true is valid, false if invalid
 */
bool NUSensorsData::getBatteryCharge(float& data)
{
    return get(BatteryCharge, data);
}

/******************************************************************************************************************************************
                                                                                                          Get Methods For Internal Use Only
 ******************************************************************************************************************************************/

/*! @brief Gets boolean sensor reading for id
    @param id the id of the sensor
    @param data will be updated with the sensor reading
    @return true if the data is valid, false otherwise
 */
bool NUSensorsData::get(const id_t& id, bool& data)
{
    const vector<int>& ids = mapIdToIndices(id);
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
    const vector<int>& ids = mapIdToIndices(id);
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
    const vector<int>& ids = mapIdToIndices(id);
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
    const vector<int>& ids = mapIdToIndices(id);
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
    const vector<int>& ids = mapIdToIndices(id);
    if (ids.size() == 1)
        return m_sensors[ids[0]].get(data);
    else
        return false;
}

/* Gets a single type of joint sensor information, eg. a Temperature with getJointData(NUSensorsData::HeadPitch, NUSensorsData::TemperatureId, data)
   @param id the id of the group of joints
   @param in the index into a joint sensor vector for the desired type of information
   @param data will be updated the the sensor readings
   @return true if the data is valid, false otherwise
 */
bool NUSensorsData::getJointData(const id_t& id, const JointSensorIndices& in, float& data)
{
    if (id < All or id > NumJointIds) 			// check that the id is actually that of a joint
        return false;
    
    const vector<int>& ids = mapIdToIndices(id);
    if (ids.size() == 1)
    {
        vector<float> vectorBuffer;
        if (m_sensors[ids[0]].get(vectorBuffer))
        {
            if (static_cast<unsigned>(in) < vectorBuffer.size())
            	data = vectorBuffer[in];
            else
                data = numeric_limits<float>::quiet_NaN();
            
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

/* Gets a vector of a single type of joint sensor information, eg. a vector of Temperatures with getJointData(NUSensorsData::RLeg, NUSensorsData:TemperatureId, data).
   @param id the id of the group of joints
   @param in the index into a joint sensor vector for the desired type of information
   @param data will be updated the the sensor readings
   @return true if the data is valid, false otherwise
 */
bool NUSensorsData::getJointData(const id_t& id, const JointSensorIndices& in, vector<float>& data)
{
    if (id < All or id > NumJointIds) 			// check that the id is actually that of a joint
        return false;
    
    const vector<int>& ids = mapIdToIndices(id);
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
            if (static_cast<unsigned>(in) < vectorBuffer.size())
                floatBuffer = vectorBuffer[in];
            else
                floatBuffer = numeric_limits<float>::quiet_NaN();
            successful &= not isnan(floatBuffer);
            data.push_back(floatBuffer);
        }
        return successful;
    }
}

/* Gets a single type of end effector information, eg. a bumper value with getEndEffectorData(NUSensorsData::LArm, NUSensorsData::BumperId, data)
   @param id the id of the end effector
   @param in the index into a end effector sensor vector for the desired type of information
   @param data will be updated the the sensor readings
   @return true if the data is valid, false otherwise
 */
bool NUSensorsData::getEndEffectorData(const id_t& id, const EndEffectorIndices& in, float& data)
{
    // map similar ids to the proper EndEffector ids
    id_t e_id = id;
    if (e_id == LArm or e_id == LHand)
        e_id = LArmEndEffector;
    else if (e_id == RArm or e_id == RHand)
        e_id = RArmEndEffector;
    else if (e_id == LLeg or e_id == LFoot)
        e_id = LLegEndEffector;
    else if (e_id == RLeg or e_id == RFoot)
        e_id = RLegEndEffector;
    
    if (e_id != LLegEndEffector and e_id != RLegEndEffector)		// check the id is valid
        return false;
    
    // proceed as usual with the proper end effector id
    const vector<int>& ids = mapIdToIndices(e_id);
    if (ids.size() == 1)
    {
        vector<float> vectorBuffer;
        if (m_sensors[ids[0]].get(vectorBuffer))
        {
            if (static_cast<size_t>(in) < vectorBuffer.size())
            	data = vectorBuffer[in];
            else
                data = numeric_limits<float>::quiet_NaN();
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

/* Gets a single type button information, eg. a button state value with getButtonData(NUSensorsData::MainButton, NUSensorsData::StateId, data)
    @param id the id of the button
    @param in the index into a button sensor vector for the desired type of information
    @param data will be updated the the sensor readings
    @return true if the data is valid, false otherwise
 */
bool NUSensorsData::getButtonData(const id_t& id, const ButtonSensorIndices& in, float& data)
{
    if (id < MainButton or id > RightButton)			// check the id is for a button sensor
        return false;
    
    const vector<int>& ids = mapIdToIndices(id);
    if (ids.size() == 1)
    {
        vector<float> vectorBuffer;
        if (m_sensors[ids[0]].get(vectorBuffer))
        {
            if (static_cast<unsigned>(in) < vectorBuffer.size())
            	data = vectorBuffer[in];
            else
                data = numeric_limits<float>::quiet_NaN();
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
    
    return get(id, data);
}

/******************************************************************************************************************************************
                                                                                                                 Convienent sub-get Methods
 ******************************************************************************************************************************************/

/*! @brief Returns true if the robot is falling over, false if it is not falling (or it is impossble to tell)
 */
bool NUSensorsData::isFalling()
{
    vector<float> temp;
    if (get(Falling, temp) and temp[0] > 0)
        return true;
    else
        return false;
}

/*! @brief Returns true if the robot has fallen over, false if it hasn't (or it is impossible to tell)
 */
bool NUSensorsData::isFallen()
{
    vector<float> temp;
    if (get(Fallen, temp) and temp[0] > 0)
        return true;
    else
        return false;
}

/*! @brief Returns true if the robot is on the ground or there are insufficient sensors to determine its state, false otherwise
 */
bool NUSensorsData::isOnGround()
{
    float lf, rf;
    if (getEndEffectorData(LFoot, ContactId, lf) and getEndEffectorData(RFoot, ContactId, rf))
    {
        if((lf > 0 or rf > 0))
        {
            return true;    // Return true if the foot sensors indicate that it is on the ground.
        }
        else
        {   
            return false;   // Return false if the foot sensors indicate that the robot is off the ground.
        } 
    }
    else    // Return true if there are no foot sensors.
    {
        return true;
    }
}

/*! @brief Returns true if the robot is currently incapacitated. A robot is incapacitated if it is falling, fallen, not on the ground or getting up
 */
bool NUSensorsData::isIncapacitated()
{
    bool gettingup = false;
    get(MotionGetupActive, gettingup);
    bool falling = isFalling();
    bool fallen = isFallen();
    bool onGround = isOnGround();
    return falling or fallen or !onGround or gettingup;
    //return isFalling() or isFallen() or not isOnGround() or gettingup;
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
    const vector<int>& ids = mapIdToIndices(id);
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
    const vector<int>& ids = mapIdToIndices(id);
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
    const vector<int>& ids = mapIdToIndices(id);
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
    const vector<int>& ids = mapIdToIndices(id);
    for (size_t i=0; i<ids.size(); i++)
        m_sensors[ids[i]].set(time, data);
}

/*! @brief Sets the readings for sensor id to be invalid 
    @param id the id of the targetted sensor
 */
void NUSensorsData::setAsInvalid(const id_t& id)
{
    const vector<int>& ids = mapIdToIndices(id);
    for (size_t i=0; i<ids.size(); i++)
        m_sensors[ids[i]].setAsInvalid();
}

/*! @brief Modifies existing sensor data. This is especially for updating 'packed' sensors.
 	@param id the id of the targetted sensor
  	@param start the index in the packed sensor the new data should be placed (This should be from the JointSensorIndices, EndEffectorIndices, ButtonSensorIndices)
 	@param time the time is ms the new data was captured
 	@param data the new data
 */
void NUSensorsData::modify(const id_t& id, int start, double time, const float& data)
{
    #if DEBUG_NUSENSORS_VERBOSITY > 4
        debug << "NUSensorsData::modify(" << id.Name << "," << time << "," << start << "," << data << ")" << endl;
    #endif
    const vector<int>& ids = mapIdToIndices(id);
    for (size_t i=0; i<ids.size(); i++)
        m_sensors[ids[i]].modify(time, start, data);
}

/*! @brief Modifies existing sensor data. This is especially for updating 'packed' sensors.
    @param id the id of the targetted sensor
    @param start the index in the packed sensor the new data should be placed (This should be from the JointSensorIndices, EndEffectorIndices, ButtonSensorIndices)
    @param time the time is ms the new data was captured
    @param data the new data
 */
void NUSensorsData::modify(const id_t& id, int start, double time, const vector<float>& data)
{
    #if DEBUG_NUSENSORS_VERBOSITY > 4
        debug << "NUSensorsData::modify(" << id.Name << "," << time << "," << start << "," << data << ")" << endl;
    #endif
    const vector<int>& ids = mapIdToIndices(id);
    size_t numids = ids.size();
    if (numids == 0)
        return;
    else if (numids == 1)
    {   // if id is a single sensor
        m_sensors[ids[0]].modify(time, start, data);
    }
    else if (numids == data.size())
    {   // if id is a group of sensors
        for (size_t i=0; i<numids; i++)
            m_sensors[ids[i]].modify(time, start, data[i]);
    }
    else
    {
        debug << "NUSensors::modify(" << id.Name << "," << time << "," << start << "," << data << "). The data is incorrectly formatted. ";
        debug << "data.size():" << data.size() << " must be ids.size():" << numids << endl;
    }
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
    (void)(output); // To stop compiler warnings.
}

/*! @brief Returns the number of sensors in the NUSensorsData
 */
int NUSensorsData::size() const
{
    return m_sensors.size();
}

void NUSensorsData::setLocSensors(const NULocalisationSensors& locsensors)
{
    int gps_id,compass_id,odom_id,falling_id,fallen_id,getup_id,lf_id,rf_id;
    const vector<int>& gps_ids = mapIdToIndices(Gps);
    if (gps_ids.size() == 1)
    {
        gps_id = gps_ids[0];
        m_sensors[gps_id] = locsensors.gps();
    }
    const vector<int>& compass_ids = mapIdToIndices(Compass);
    if (compass_ids.size() == 1)
    {
        compass_id = compass_ids[0];
        m_sensors[compass_id] = locsensors.compass();
    }
    const vector<int>& odom_ids = mapIdToIndices(Odometry);
    if (odom_ids.size() == 1)
    {
        odom_id = odom_ids[0];
        m_sensors[odom_id] = locsensors.odometry();
    }
    const vector<int>& falling_ids = mapIdToIndices(Falling);
    if (falling_ids.size() == 1)
    {
        falling_id = falling_ids[0];
        m_sensors[falling_id] = locsensors.falling();
    }
    const vector<int>& fallen_ids = mapIdToIndices(Fallen);
    if (fallen_ids.size() == 1)
    {
        fallen_id = fallen_ids[0];
        m_sensors[fallen_id] = locsensors.fallen();
    }
    const vector<int>& getup_ids = mapIdToIndices(MotionGetupActive);
    if (fallen_ids.size() == 1)
    {
        getup_id = getup_ids[0];
        m_sensors[getup_id] = locsensors.getup();
    }
    const vector<int>& lf_ids = mapIdToIndices(LLegEndEffector);
    if (lf_ids.size() == 1)
    {
        lf_id = lf_ids[0];
        m_sensors[lf_id] = locsensors.leftFoot();
    }
    const vector<int>& rf_ids = mapIdToIndices(RLegEndEffector);
    if (rf_ids.size() == 1)
    {
        rf_id = rf_ids[0];
        m_sensors[rf_id] = locsensors.rightFoot();
    }
    CurrentTime = locsensors.GetTimestamp();
    return;
}

NULocalisationSensors NUSensorsData::getLocSensors()
{
    int gps_id,compass_id,odom_id,falling_id,fallen_id,getup_id,lf_id,rf_id;
    const vector<int>& gps_ids = mapIdToIndices(Gps);
    if (gps_ids.size() == 1)
    {
        gps_id = gps_ids[0];
    }
    const vector<int>& compass_ids = mapIdToIndices(Compass);
    if (compass_ids.size() == 1)
    {
        compass_id = compass_ids[0];
    }
    const vector<int>& odom_ids = mapIdToIndices(Odometry);
    if (odom_ids.size() == 1)
    {
        odom_id = odom_ids[0];
    }
    const vector<int>& falling_ids = mapIdToIndices(Falling);
    if (falling_ids.size() == 1)
    {
        falling_id = falling_ids[0];
    }
    const vector<int>& fallen_ids = mapIdToIndices(Fallen);
    if (fallen_ids.size() == 1)
    {
        fallen_id = fallen_ids[0];
    }

    const vector<int>& getup_ids = mapIdToIndices(MotionGetupActive);
    if (fallen_ids.size() == 1)
    {
        getup_id = getup_ids[0];
    }
    const vector<int>& lf_ids = mapIdToIndices(LLegEndEffector);
    if (lf_ids.size() == 1)
    {
        lf_id = lf_ids[0];
    }
    const vector<int>& rf_ids = mapIdToIndices(RLegEndEffector);
    if (rf_ids.size() == 1)
    {
        rf_id = rf_ids[0];
    }

    return NULocalisationSensors(GetTimestamp(), m_sensors[gps_id], m_sensors[compass_id], m_sensors[odom_id], m_sensors[falling_id],
                                 m_sensors[fallen_id], m_sensors[getup_id], m_sensors[lf_id], m_sensors[rf_id]);
}

/*! @brief Put the entire contents of the NUSensorsData class into a stream
 */
ostream& operator<< (ostream& output, const NUSensorsData& p_data)
{    
    output << p_data.m_common_ids << endl;
    output << p_data.m_ids_copy << endl;
    output << p_data.m_id_to_indices << endl;
    output << p_data.m_available_ids << endl;
    output << p_data.size() << " ";
    for (int i=0; i<p_data.size(); i++)
        output << p_data.m_sensors[i] << endl;
    return output;
}

/*! @brief Get the entire contents of the NUSensorsData class from a stream
 */
istream& operator>> (istream& input, NUSensorsData& p_data)
{
    //input >> p_data.m_common_ids;
    readIdList(input, p_data.m_common_ids);
    //input >> p_data.m_ids_copy;
    readIdList(input, p_data.m_ids_copy);
    input >> p_data.m_id_to_indices;
    input >> p_data.m_available_ids;
    p_data.m_sensors.clear();
    int numsensors;
    input >> numsensors;
    double lastUpdateTime = 0;
    Sensor tempSensor("temp");
    for (int i=0; i<numsensors; i++)
    {
        if(!input.good()) throw exception();
        input >> tempSensor;
        p_data.m_sensors.push_back(Sensor(tempSensor));
        if(tempSensor.Time > lastUpdateTime) lastUpdateTime = tempSensor.Time;
    }
    p_data.CurrentTime = lastUpdateTime;
    //force eofbit
    input.ignore(128, '\n');
    input.peek();
    return input;
}

void readIdList(istream& input, std::vector<NUData::id_t*>& list)
{
    stringstream wholevector;
    //list.clear();
    // get all of the data between [ ... ]
    input.ignore(128, '[');
    char c;
    int brackets = 1;
    while (brackets != 0 and input.good())
    {
        input.get(c);
        wholevector << c;
        if (c == '[')
            brackets++;
        else if (c == ']')
            brackets--;
    }

    NUData::id_t* buffer;
    unsigned int counter = 0;
    // now split the data based on the commas
    while (wholevector.peek() != ']' and wholevector.good())
    {
        buffer = list[counter++];
        wholevector >> buffer;
        wholevector.ignore(128, ',');
        //list.push_back(buffer);
    }
}

