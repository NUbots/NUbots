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

int s_curr_id = NUData::NumCommonIds.Id;
vector<NUSensorsData::id_t*> NUSensorsData::m_ids;

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
// foot sensors
const NUSensorsData::id_t NUSensorsData::FootBumper(s_curr_id++, "FootBumper", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::FootForce(s_curr_id++, "FootForce", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::FootContact(s_curr_id++, "FootContact", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::FootSupport(s_curr_id++, "FootSupport", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::FootImpact(s_curr_id++, "FootImpact", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::FootCoP(s_curr_id++, "FootCoP", NUSensorsData::m_ids);
// button sensors
const NUSensorsData::id_t NUSensorsData::MainButton(s_curr_id++, "MainButton", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::SecondaryButton(s_curr_id++, "SecondaryButton", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::AllButton(s_curr_id++, "AllButton", NUSensorsData::m_ids);
const NUSensorsData::id_t NUSensorsData::AllButtonTriggers(s_curr_id++, "AllButtonTriggers", NUSensorsData::m_ids);
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
    addDevices(hardwarenames);
    // That call is not going to add most of the sensors that you want, I could make it so that it would, but I don't think I want it to.
    // I don't think it is really necessary to have this at all.
}

/******************************************************************************************************************************************
                                                                                                                                Get Methods
 ******************************************************************************************************************************************/

/*! @brief Gets the requested joint position. If the operation is successful true is returned, 
           otherwise false is returned and position is unchanged
    @param jointid the id of the joint you want the data for
    @param position the position will be placed in this variable
 */
bool NUSensorsData::getJointPosition(id_t jointid, float& position)
{
    return false;
}

/*! @brief Gets the requested joint velocity. If the operation is successful true is returned, 
           otherwise false is returned and velocity is unchanged
    @param jointid the id of the joint you want the data for
    @param velocity the velocity will be placed in this variable
 */
bool NUSensorsData::getJointVelocity(id_t jointid, float& velocity)
{
    return false;
}

/*! @brief Gets the requested joint acceleration. If the operation is successful true is returned, 
           otherwise false is returned and acceleration is unchanged
    @param jointid the id of the joint you want the data for
    @param acceleration the acceleration will be placed in this variable
 */
bool NUSensorsData::getJointAcceleration(id_t jointid, float& acceleration)
{
    return false;
}

/*! @brief Gets the requested joint target. If the operation is successful true is returned, 
           otherwise false is returned and target is unchanged
    @param jointid the id of the joint you want the data for
    @param target the target will be placed in this variable
 */
bool NUSensorsData::getJointTarget(id_t jointid, float& target)
{
    return false;
}

/*! @brief Gets the requested joint stiffness. If the operation is successful true is returned, 
           otherwise false is returned and stiffness is unchanged
    @param jointid the id of the joint you want the data for
    @param stiffness the stiffness will be placed in this variable
 */
bool NUSensorsData::getJointStiffness(id_t jointid, float& stiffness)
{
    return false;
}

/*! @brief Gets the requested joint current. If the operation is successful true is returned, 
           otherwise false is returned and current is unchanged
    @param jointid the id of the joint you want the data for
    @param current the current will be placed in this variable
 */
bool NUSensorsData::getJointCurrent(id_t jointid, float& current)
{
    return false;
}

/*! @brief Gets the requested joint torque. If the operation is successful true is returned, 
           otherwise false is returned and torque is unchanged
    @param jointid the id of the joint you want the data for
    @param torque the torque will be placed in this variable
 */
bool NUSensorsData::getJointTorque(id_t jointid, float& torque)
{
    return false;
}

/*! @brief Gets the requested joint temperatures. If the operation is successful true is returned, 
           otherwise false is returned and temperatures is unchanged
    @param jointid the id of the joint you want the data for
    @param temperature the temperature will be placed in this variable
 */
bool NUSensorsData::getJointTemperature(id_t jointid, float& temperature)
{
    return false;
}

/*! @brief Returns the number of joints in the specified body part
 @param partid the id of the body part
 @return the number of joints
 */
int NUSensorsData::getNumberOfJoints(id_t partid)
{
    return 0;
}

/*! @brief Gets the requested joint positions in a given body part. If the get is successful true is returned
 otherwise false is returned, and the input variable is left unchanged.
 @param bodypart the id of the body part to want the data for
 @param positions the input vector that will be updated to contain the requested data
 */
bool NUSensorsData::getJointPositions(id_t bodypart, vector<float>& positions)
{
    return false;
}

/*! @brief Gets the requested joint velocities in a given body part. If the get is successful true is returned
 otherwise false is returned, and the input variable is left unchanged.
 @param bodypart the id of the body part to want the data for
 @param velocities the input vector that will be updated to contain the requested data
 */
bool NUSensorsData::getJointVelocities(id_t bodypart, vector<float>& velocities)
{
    return false;
}

/*! @brief Gets the requested joint accelerations in a given body part. If the get is successful true is returned
 otherwise false is returned, and the input variable is left unchanged.
 @param bodypart the id of the body part to want the data for
 @param accelerations the input vector that will be updated to contain the requested data
 */
bool NUSensorsData::getJointAccelerations(id_t bodypart, vector<float>& accelerations)
{
    return false;
}

/*! @brief Gets the requested joint targets in a given body part. If the get is successful true is returned
 otherwise false is returned, and the input variable is left unchanged.
 @param bodypart the id of the body part to want the data for
 @param targets the input vector that will be updated to contain the requested data
 */
bool NUSensorsData::getJointTargets(id_t bodypart, vector<float>& targets)
{
    return false;
}

/*! @brief Gets the requested joint stiffnesses in a given body part. If the get is successful true is returned
 otherwise false is returned, and the input variable is left unchanged.
 @param bodypart the id of the body part to want the data for
 @param stiffnesses the input vector that will be updated to contain the requested data
 */
bool NUSensorsData::getJointStiffnesses(id_t bodypart, vector<float>& stiffnesses)
{
    return false;
}

/*! @brief Gets the requested joint currents in a given body part. If the get is successful true is returned
 otherwise false is returned, and the input variable is left unchanged.
 @param bodypart the id of the body part to want the data for
 @param currents the input vector that will be updated to contain the requested data
 */
bool NUSensorsData::getJointCurrents(id_t bodypart, vector<float>& currents)
{
    return false;
}

/*! @brief Gets the requested joint torques in a given body part. If the get is successful true is returned
 otherwise false is returned, and the input variable is left unchanged.
 @param bodypart the id of the body part to want the data for
 @param torques the input vector that will be updated to contain the requested data
 */
bool NUSensorsData::getJointTorques(id_t bodypart, vector<float>& torques)
{
    return false;
}

/*! @brief Gets the requested joint temperatures in a given body part. If the get is successful true is returned
 otherwise false is returned, and the input variable is left unchanged.
 @param bodypart the id of the body part to want the data for
 @param temperatures the input vector that will be updated to contain the requested data
 */
bool NUSensorsData::getJointTemperatures(id_t bodypart, vector<float>& temperatures)
{
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

/*! @brief Gets the odometry data since the last call
    @param time the time of the last call
    @param values will be updated with the odometry [x (cm), y(cm), yaw(rad)] since time
 */
bool NUSensorsData::getOdometry(float& time, vector<float>& values)
{
    return false;
    /*
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
    }*/
}

bool NUSensorsData::getOdometryData(vector<float>& values)
{
    return false;
}

/* @brief Gets the height of the camera off the ground in cm
   @param height will be updated with the height of the camera from the ground
 */
bool NUSensorsData::getCameraHeight(float& height)
{
    return false;
    /*
    if (CameraHeight == NULL || CameraHeight->IsValid == false)
        return false;
    else
    {
        height = CameraHeight->Data[0];
        return true;
    }*/
}

/*! @brief Gets the names of the joints in a particular body part.
    @param partid the id of the body part 
    @param names will be updated to contain the names of the joints in that body part
 */
bool NUSensorsData::getJointNames(id_t partid, vector<string>& names)
{
    /*vector<id_t> selectedjoints;
    if (partid == All)
        selectedjoints = m_all_joint_ids;
    if (partid == Body)
        selectedjoints = m_body_ids;
    else if (partid == Head)
        selectedjoints = m_head_ids;
    else if (partid == LArm)
        selectedjoints = m_larm_ids;
    else if (partid == RArm)
        selectedjoints = m_rarm_ids;
    else if (partid == Torso)
        selectedjoints = m_torso_ids;
    else if (partid == LLeg)
        selectedjoints = m_lleg_ids;
    else if (partid == RLeg)
        selectedjoints = m_rleg_ids;
    else
    {
        errorlog << "NUSensorsData::getJointNames. UNDEFINED Body part.";
        return false;
    }
    
    names.clear();
    for (unsigned int i=0; i<selectedjoints.size(); i++)
        names.push_back(selectedjoints[i].Name);*/
    return false;
}

/*! @brief Gets the accelerometer values [ax, ay, az] in cm/s/s
    @param values will be updated with the current accelerometer readings
 */
bool NUSensorsData::getAccelerometerValues(vector<float>& values)
{
    return false;
    /*
    if (BalanceAccelerometer == NULL || BalanceAccelerometer->IsValid == false)
        return false;
    else
    {
        values = BalanceAccelerometer->Data;
        return true;
    }*/
}

/*! @brief Gets the accelerometer values [ax, ay, az] in cm/s/s
    @param values will be updated with the current accelerometer readings
 */
bool NUSensorsData::getHorizon(vector<float>& values)
{
    return false;
    /*
    if (BalanceHorizon == NULL || BalanceHorizon->IsValid == false)
        return false;
    else
    {
        values = BalanceHorizon->Data;
        return true;
    }*/
}

/*! @brief Gets the last button trigger times for the chest button, left
            foot bumper and right foot bumper.
    @param values will be updated with the current utton trigger times.
 */
bool NUSensorsData::getButtonTriggers(vector<float>& values)
{
    return false;
    /*
    if (ButtonTriggers == NULL || ButtonTriggers->IsValid == false)
        return false;
    else
    {
        values = ButtonTriggers->Data;
        return true;
    }*/
}

/*! @brief Gets the gyro values [gx, gy, gz] in rad/s
    @param values will be updated with the current gyro readings
 */
bool NUSensorsData::getGyroValues(vector<float>& values)
{
    return false;
    /*
    if (BalanceGyro == NULL || BalanceGyro->IsValid == false)
        return false;
    else
    {
        values = BalanceGyro->Data;
        return true;
    }*/
}

/*! @brief Gets the gyro offset values [offsetx, offsety, offsetz]
    @param values will be updated with the current estimate of the gyro offset
    @return returns true if the values are valid false otherwise
 */
bool NUSensorsData::getGyroOffsetValues(vector<float>& values)
{
    return false;
    /*
    if (BalanceGyroOffset == NULL || BalanceGyroOffset->IsValid == false)
        return false;
    else
    {
        values = BalanceGyroOffset->Data;
        return true;
    }
     */
}

/*! @brief Gets the gyro values after the offset and filtering is applied [gx, gy, gz]
    @param values will be updated with the current estimate of the gyro
    @return returns true if the values are valid false otherwise
 */
bool NUSensorsData::getGyroFilteredValues(vector<float>& values)
{
    return false;
    /*
    if (BalanceGyro == NULL || BalanceGyro->IsValid == false || BalanceGyroOffset == NULL || BalanceGyroOffset->IsValid == false)
        return false;
    else
    {
        values = vector<float>(3,0);
        for (size_t i=0; i<values.size(); i++)
            values[i] = BalanceGyro->Data[i] - BalanceGyroOffset->Data[i];
        return true;
    }*/
}

/*! @brief Gets the orientation [roll, pitch, yaw] in radians of the robot's torso
    @param values will be updated with the current orientation estimate
 */
bool NUSensorsData::getOrientation(vector<float>& values)
{
    return false;
    /*
    if (BalanceOrientation == NULL || BalanceOrientation->IsValid == false)
        return false;
    else 
    {
        values = BalanceOrientation->Data;
        return true;
    }*/
}

/*! @brief Gets the orientation [roll, pitch, yaw] in radians from the robot's hardware orientation sensor
 */
bool NUSensorsData::getOrientationHardware(vector<float>& values)
{
    return false;
    /*
    if (BalanceOrientationHardware == NULL || BalanceOrientationHardware->IsValid == false)
        return false;
    else 
    {
        values = BalanceOrientationHardware->Data;
        return true;
    }*/
}

/*! @brief Gets the zero moment point [x,y] in cm from somewhere?
    @param values will be updated with the current ZMP estimate
 */
bool NUSensorsData::getZMP(vector<float>& values)
{
    return false;
    /*
    if (BalanceZMP == NULL || BalanceZMP->IsValid == false)
        return false;
    else 
    {
        values = BalanceZMP->Data;
        return true;
    }*/
}

/*! @brief Gets the falling sense [sum, left, right, forward, backward] 
    @param values will be updated with the current falling measurements [sum, left, right, forward, backward]
 */
bool NUSensorsData::getFalling(vector<float>& values)
{
    return false;
    /*
    if (BalanceFalling == NULL || BalanceFalling->IsValid == false)
        return false;
    else 
    {
        values = BalanceFalling->Data;
        return true;
    }*/
}

/*! @brief Gets the fallen sense [sum, left, right, forward, backward] 
    @param values will be updated with the current fallen measurements [sum, left, right, forward, backward]
 */
bool NUSensorsData::getFallen(vector<float>& values)
{
    return false;
    /*
    if (BalanceFallen == NULL || BalanceFallen->IsValid == false)
        return false;
    else 
    {
        values = BalanceFallen->Data;
        return true;
    }*/
}

/*! @brief Gets the distance sensor readings (sensors from left to right) in centimeters
    @param values will be updated with the current distance readings
 */
bool NUSensorsData::getDistanceLeftValues(vector<float>& values)
{
    return false;
    /*
    if (DistanceLeftValues == NULL || DistanceLeftValues->IsValid == false)
        return false;
    else
    {
        values = DistanceLeftValues->Data;
        return true;
    }*/
}

bool NUSensorsData::getDistanceRightValues(vector<float>& values)
{
    return false;
    /*
    if (DistanceRightValues == NULL || DistanceRightValues->IsValid == false)
        return false;
    else
    {
        values = DistanceRightValues->Data;
        return true;
    }*/
}

/*! @brief Gets the battery readings [voltage (V), current (A), charge (%)]
    @param values will be updated with the current battery sensor values
 */
bool NUSensorsData::getBatteryValues(vector<float>& values)
{
    return false;
    /*
    if (BatteryValues == NULL || BatteryValues->IsValid == false)
        return false;
    else
    {
        values = BatteryValues->Data;
        return true;
    }*/
}

/*! @brief Gets the GPS readings [x (cm), y(cm), z (cm)]
    @param values will be updated with the gps coordinates of the robot [x (cm), y (cm), z (cm)]
 */
bool NUSensorsData::getGPSValues(vector<float>& values)
{
    return false;
    /*
    if (GPS == NULL || GPS->IsValid == false)
        return false;
    else
    {
        values = GPS->Data;
        return true;
    }*/
}

/*! @brief Gets the compass reading [heading]
    @param values will be updated with the compass reading
 */
bool NUSensorsData::getCompassValues(vector<float>& values)
{
    return false;
    /*
    if (Compass == NULL || Compass->IsValid == false)
        return false;
    else
    {
        values = Compass->Data;
        return true;
    }*/
}

/*! @brief Gets the foot sole pressure sensor values (order: left to right front to back) in Newtons
    @param footid the id of the part of the foot you want the readings for
    @param values will be updated with the current readings for the selected foot
 */
bool NUSensorsData::getFootSoleValues(id_t footid, vector<float>& values)
{
    return false;
    /*
    if (FootSoleValues == NULL || FootSoleValues->IsValid == false)
        return false;
    else
    {
        int numfootsolesensors = FootSoleValues->Data.size();
        if (footid == All)
            values = FootSoleValues->Data;
        else if (footid == LLeg)
        {
            vector<float> leftfootvalues(numfootsolesensors/2, 0);
            for (unsigned int i=0; i<leftfootvalues.size(); i++)
                leftfootvalues[i] = FootSoleValues->Data[i];
            values = leftfootvalues;
        }
        else if (footid == RLeg)
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
    }*/
}

/*! @brief Gets the centre of pressure as measured by feet sensors
 
    If a single foot is requested the x and y positions are relative to the position of the ankle on the foot.
    If the CoP for both feet is requested the x and y positions are relative to the torso
 
    @param footid LeftFoot, RightFoot, AllFeet
    @param x the distance in cm forwards 
    @param y the distance in cm backwards
 */
bool NUSensorsData::getFootCoP(id_t footid, float& x, float& y)
{
    return false;
    /*
    if (FootCoP == NULL || FootCoP->IsValid == false)
        return false;
    else
    {
        if (footid == LLeg)
        {
            x = (*FootCoP)[0];
            y = (*FootCoP)[1];
        }
        else if (footid == RLeg)
        {
            x = (*FootCoP)[2];
            y = (*FootCoP)[3];
        }
        else if (footid == All)
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
    }*/
}

/*! @brief Gets the foot bumper sensor values (order: left to right) in binary (0=off 1=on)
    @param footid the id of the part of the foot you want the readings for
    @param values will be updated with the current readings for the selected foot
 */
bool NUSensorsData::getFootBumperValues(id_t footid, vector<float>& values)
{
    return false;
    /*
    if (FootBumperValues == NULL || FootBumperValues->IsValid == false)
        return false;
    else
    {
        int numfootbumpersensors = FootBumperValues->Data.size();
        if (footid == All)
            values = FootBumperValues->Data;
        else if (footid == LLeg)
        {
            vector<float> leftfootvalues(numfootbumpersensors/2, 0);
            for (unsigned int i=0; i<leftfootvalues.size(); i++)
                leftfootvalues[i] = FootBumperValues->Data[i];
            values = leftfootvalues;
        }
        else if (footid == RLeg)
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
    }*/
}

/*! @brief Gets the total force on the foot in Newtons
    @param footid the id of the part of the foot you want the readings for
    @param force will be updated with the current readings for the selected foot
 */
bool NUSensorsData::getFootForce(id_t footid, float& force)
{
    return false;
    /*
    force = 0;
    if (FootForce == NULL || FootForce->IsValid == false)
        return false;
    else
    {
        if (footid == LLeg)
        {
            force = (*FootForce)[0];
        }
        else if (footid == RLeg)
        {
            force = (*FootForce)[1];
        }
        else if (footid == All)
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
    }*/
}

/*! @brief Gets the whether footid is in contact with the ground
    @param footid the id of the foot
    @param contact will be updated to true if the foot is on the ground, false if it is not on the ground
 */
bool NUSensorsData::getFootContact(id_t footid, bool& contact)
{
    return false;
    /*
    if (FootContact == NULL || FootContact->IsValid == false)
        return false;
    else
    {
        if (footid == LLeg)
            contact = (*FootContact)[0];
        else if (footid == RLeg)
            contact = (*FootContact)[1];
        else if (footid == All)
            contact = (*FootContact)[2];
        else
        {
            debug << "NUSensorsData::getFootContact(). Unknown foot id." << endl;
            return false;
        }
        return true;
    }*/
}

/*! @brief Gets the total force on the foot in Newtons
    @param footid the id of the part of the foot you want the readings for
    @param support will be updated to true if footid is supporting the robot.
    @return true if support was updated, false if invalid
 */
bool NUSensorsData::getFootSupport(id_t footid, bool& support)
{
    return false;
    /*
    if (FootSupport == NULL || FootSupport->IsValid == false)
        return false;
    else
    {
        if (footid == LLeg)
            support = static_cast<bool> ((*FootSupport)[0]);
        else if (footid == RLeg)
            support = static_cast<bool> ((*FootSupport)[1]);
        else if (footid == All)
            support = static_cast<bool> ((*FootSupport)[0]) and static_cast<bool> ((*FootSupport)[1]);
        else
        {
            debug << "NUSensorsData::getFootForce(). Unknown foot id." << endl;
            return false;
        }
        return true;
    }*/
}

/*! @brief Gets the button values (order: importance) in binary (0=off 1=on)
    @param buttonid the id of the button(s) you want the readings for
    @param values will be updated with the current readings for the selected button(s)
 */
bool NUSensorsData::getButtonValues(id_t buttonid, vector<float>& values)
{
    return false;
    /*
    if (ButtonValues == NULL || ButtonValues->IsValid == false)
        return false;
    else
    {
        if (buttonid == AllButton)
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
    }*/
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

/*! @brief Get whether the fall motion module is active
    @param active will be updated to true if the fall module is active
    @return true if the data is valid
 */
bool NUSensorsData::getMotionFallActive(bool& active)
{
    return false;
    /*
    if (not MotionFallActive->IsValid)
        return false;
    else
    {
        active = MotionFallActive->Data[0];
        return true;
    }*/
}

/*! @brief Get whether the getup motion module is active
    @param active will be updated to true if the getup module is active
    @return true if the data is valid
 */
bool NUSensorsData::getMotionGetupActive(bool& active)
{
    return false;
    /*
    if (not MotionGetupActive->IsValid)
        return false;
    else
    {
        active = MotionGetupActive->Data[0];
        return true;
    }*/
}

/*! @brief Get whether the kick motion module is active
    @param active will be updated to true if the kick module is active
    @return true if the data is valid
 */
bool NUSensorsData::getMotionKickActive(bool& active)
{
    return false;
    /*
    if (not MotionKickActive->IsValid)
        return false;
    else
    {
        active = MotionKickActive->Data[0];
        return true;
    }*/
}

/*! @brief Get whether the save motion module is active
    @param active will be updated to true if the save module is active
    @return true if the data is valid
 */
bool NUSensorsData::getMotionSaveActive(bool& active)
{
    return false;
    /*
    if (not MotionSaveActive->IsValid)
        return false;
    else
    {
        active = MotionSaveActive->Data[0];
        return true;
    }*/
}

/*! @brief Get whether the script motion module is active
    @param active will be updated to true if the script module is active
    @return true if the data is valid
 */
bool NUSensorsData::getMotionScriptActive(bool& active)
{
    return false;
    /*
    if (not MotionSaveActive->IsValid)
        return false;
    else
    {
        active = MotionSaveActive->Data[0];
        return true;
    }*/
}

/*! @brief Get current walk speed [cm/s, cm/s, rad/s]
    @param speed will be updated with the current commanded walk speed
    @return true if the data is valid
 */
bool NUSensorsData::getMotionWalkSpeed(vector<float>& speed)
{
    return false;
    /*
    if (not MotionWalkSpeed->IsValid)
        return false;
    else
    {
        speed = MotionWalkSpeed->Data;
        return true;
    }*/
}

/*! @brief Gets the current walk maximum speed as [x cm/s, y cm/s yaw rad/s]
    @param speed will be updated with the current max walk speed
    @return true if the data is valid
 */
bool NUSensorsData::getMotionWalkMaxSpeed(vector<float>& speed)
{
    return false;
    /*
    if (not MotionWalkMaxSpeed->IsValid)
        return false;
    else
    {
        speed = MotionWalkMaxSpeed->Data;
        return true;
    }*/
}

/*! @brief Get the current head completion time 
    @param time will be updated with the current head completion time
    @return true if the data is valid
 */
bool NUSensorsData::getMotionHeadCompletionTime(double& time)
{
    return false;
    /*
    if (not MotionHeadCompletionTime->IsValid)
        return false;
    else
    {
        time = MotionHeadCompletionTime->Data[0];
        return true;
    }*/
}

/******************************************************************************************************************************************
                                                                                                                                Set Methods
 ******************************************************************************************************************************************/

void NUSensorsData::set(const id_t& id, double time, const float& data)
{
}

void NUSensorsData::set(const id_t& id, double time, const vector<float>& data)
{
}

void NUSensorsData::set(const id_t& id, double time, const vector<vector<float> >& data)
{
}

void NUSensorsData::set(const id_t& id, double time, const string& data)
{
}

void NUSensorsData::setAsInvalid(const id_t& id)
{
}

void NUSensorsData::setVelocity(const id_t& id, double time, const vector<float>& data)
{
}

void NUSensorsData::setAcceleration(const id_t& id, double time, const vector<float>& data)
{
}

void NUSensorsData::setTarget(const id_t& id, double time, const vector<float>& data)
{
}

void NUSensorsData::setStiffness(const id_t& id, double time, const vector<float>& data)
{
}

void NUSensorsData::setCurrent(const id_t& id, double time, const vector<float>& data)
{
}

void NUSensorsData::setTemperature(const id_t& id, double time, const vector<float>& data)
{
}

/*! @brief Sets the joint positions to the given values
    @param time the time the data was collected in milliseconds
    @param data the new joint position values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setJointPositions(double time, const vector<float>& data, bool iscalculated)
{
    //setData(JointPositions, time, data, iscalculated);
}

/*! @brief Sets the joint velocities to the given values
     @param time the time the data was collected in milliseconds
     @param data the new joint velocities values
     @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setJointVelocities(double time, const vector<float>& data, bool iscalculated)
{
    //setData(JointVelocities, time, data, iscalculated);
}

/*! @brief Sets the joint accelerations to the given values
    @param time the time the data was collected in milliseconds
    @param data the new joint accelerations values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setJointAccelerations(double time, const vector<float>& data, bool iscalculated)
{
    //setData(JointAccelerations, time, data, iscalculated);
}

/*! @brief Sets the joint targets to the given values
    @param time the time the data was collected in milliseconds
    @param data the new joint targets values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setJointTargets(double time, const vector<float>& data, bool iscalculated)
{
    //setData(JointTargets, time, data, iscalculated);
}

/*! @brief Sets the joint stiffnesses to the given values
    @param time the time the data was collected in milliseconds
    @param data the new joint stiffnesses values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setJointStiffnesses(double time, const vector<float>& data, bool iscalculated)
{
    //setData(JointStiffnesses, time, data, iscalculated);
}

/*! @brief Sets the joint currents to the given values
    @param time the time the data was collected in milliseconds
    @param data the new joint currents values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setJointCurrents(double time, const vector<float>& data, bool iscalculated)
{
    //setData(JointCurrents, time, data, iscalculated);
}

/*! @brief Sets the joint torques to the given values
    @param time the time the data was collected in milliseconds
    @param data the new joint torques values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setJointTorques(double time, const vector<float>& data, bool iscalculated)
{
    //setData(JointTorques, time, data, iscalculated);
}

/*! @brief Sets the joint temperature to the given values
    @param time the time the data was collected in milliseconds
    @param data the new joint temperature values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setJointTemperatures(double time, const vector<float>& data, bool iscalculated)
{
    //setData(JointTemperatures, time, data, iscalculated);
}

/*! @brief Sets the accelerometer to the given values
    @param time the time the data was collected in milliseconds
    @param data the new accelerometer values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setBalanceAccelerometer(double time, const vector<float>& data, bool iscalculated)
{
    //setData(BalanceAccelerometer, time, data, iscalculated);
}

/*! @brief Sets the gyro to the given values
    @param time the time the data was collected in milliseconds
    @param data the gyro values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setBalanceGyro(double time, const vector<float>& data, bool iscalculated)
{
    //setData(BalanceGyro, time, data, iscalculated);
}

/*! @brief Sets the hardware measurement of the robot's orientation
    @param time the time the data was collected in milliseconds
    @param data the orientation measurement
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setBalanceOrientationHardware(double time, const vector<float>& data, bool iscalculated)
{
    //setData(BalanceOrientationHardware, time, data, iscalculated);
}

/*! @brief Sets the left distance values to the given values
    @param time the time the data was collected in milliseconds
    @param data the new distance values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setDistanceLeftValues(double time, const vector<float>& data, bool iscalculated)
{
    //setData(DistanceLeftValues, time, data, iscalculated);
}

/*! @brief Sets the right distance values to the given values
    @param time the time the data was collected in milliseconds
    @param data the new distance values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setDistanceRightValues(double time, const vector<float>& data, bool iscalculated)
{
    //setData(DistanceRightValues, time, data, iscalculated);
}

/*! @brief Sets the foot sole values to the given values
 @param time the time the data was collected in milliseconds
 @param data the new foot sole values values
 @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setFootSoleValues(double time, const vector<float>& data, bool iscalculated)
{
    //setData(FootSoleValues, time, data, iscalculated);
}

/*! @brief Sets the foot bumper to the given values
    @param time the time the data was collected in milliseconds
    @param data the new foot bumper values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setFootBumperValues(double time, const vector<float>& data, bool iscalculated)
{
    //setData(FootBumperValues, time, data, iscalculated);
}

/*! @brief Sets the button values to the given values
    @param time the time the data was collected in milliseconds
    @param data the new button values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setButtonValues(double time, const vector<float>& data, bool iscalculated)
{
    //setData(ButtonValues, time, data, iscalculated);
}

/*! @brief Sets the battery to the given values
    @param time the time the data was collected in milliseconds
    @param data the battery values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setBatteryValues(double time, const vector<float>& data, bool iscalculated)
{
    //setData(BatteryValues, time, data, iscalculated);
}

/*! @brief Sets whether the fall engine is active
    @param time the timestamp
    @param active true if fall is active
 */ 
void NUSensorsData::setMotionFallActive(double time, bool active)
{
    //vector<float> data(1,0);
    //data[0] = active;
    //setData(MotionFallActive, time, data, false);
}

/*! @brief Sets whether the getup engine is active
    @param time the timestamp
    @param active true if getup is active
 */ 
void NUSensorsData::setMotionGetupActive(double time, bool active)
{
    //vector<float> data(1,0);
    //data[0] = active;
    //setData(MotionGetupActive, time, data, false);
}

/*! @brief Sets whether the kick engine is active
    @param time the timestamp
    @param active true if kick is active
 */ 
void NUSensorsData::setMotionKickActive(double time, bool active)
{
    //vector<float> data(1,0);
    //data[0] = active;
    //setData(MotionKickActive, time, data, false);
}

/*! @brief Sets whether the save engine is active
    @param time the timestamp
    @param active true if save is active
 */ 
void NUSensorsData::setMotionSaveActive(double time, bool active)
{
    //vector<float> data(1,0);
    //data[0] = active;
    //setData(MotionSaveActive, time, data, false);
}

/*! @brief Sets whether the script engine is active
    @param time the timestamp
    @param active true if script is active
 */
void NUSensorsData::setMotionScriptActive(double time, bool active)
{
    //vector<float> data(1,0);
    //data[0] = active;
    //setData(MotionScriptActive, time, data, false);
}

/*! @brief Sets the current walk speed 
    @param time the timestamp
    @param speed [x,y,yaw]
 */
void NUSensorsData::setMotionWalkSpeed(double time, vector<float>& speed)
{
    //setData(MotionWalkSpeed, time, speed, false);
}

/*! @brief Sets the current walk speed 
    @param time the timestamp
    @param speed the maximum speeds of [x,y,yaw]
 */
void NUSensorsData::setMotionWalkMaxSpeed(double time, vector<float>& speed)
{
    //setData(MotionWalkMaxSpeed, time, speed, false);
}

/*! @brief Sets the completion time of the current head movement
    @param time the time the data was set in milliseconds
    @param completiontime the head movement completion time
 */
void NUSensorsData::setMotionHeadCompletionTime(double time, double completiontime)
{
    //vector<float> ct(1,0);
    //ct[0] = completiontime;
    //setData(MotionHeadCompletionTime, time, ct, false);
}

/*! @brief Sets the GPS coordinates to the given values
    @param time the time the data was collected in milliseconds
    @param data the GPS values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setGPSValues(double time, const vector<float>& data, bool iscalculated)
{
    //setData(GPS, time, data, iscalculated);
}

/*! @brief Sets the compass coordinates to the given values
    @param time the time the data was collected in milliseconds
    @param data the Compass values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setCompassValues(double time, const vector<float>& data, bool iscalculated)
{
    //setData(Compass, time, data, iscalculated);
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


