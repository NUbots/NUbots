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
#include "Tools/debug.h"

#include <fstream>
#include <cctype>       // for tolower()

joint_id_t NUSensorsData::HeadYaw = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::HeadPitch = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::LShoulderPitch = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::LShoulderRoll = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::LElbowYaw = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::LElbowRoll = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::RShoulderPitch = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::RShoulderRoll = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::RElbowYaw = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::RElbowRoll = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::LHipYawPitch = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::LHipPitch = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::LHipRoll = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::LKneePitch = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::LAnklePitch = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::LAnkleRoll = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::RHipYawPitch = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::RHipPitch = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::RHipRoll = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::RKneePitch = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::RAnklePitch = NUSensorsData::SENSOR_MISSING;
joint_id_t NUSensorsData::RAnkleRoll = NUSensorsData::SENSOR_MISSING;

/*!
 */
NUSensorsData::NUSensorsData()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensorsData::NUSensorsData" << endl;
#endif
    
    // create the sensor_t's
    addSensor(&JointPositions, string("JointPositions"), JOINT_POSITIONS);
    addSensor(&JointVelocities, string("JointVelocities"), JOINT_VELOCITIES);
    addSensor(&JointAccelerations, string("JointAccelerations"), JOINT_ACCELERATIONS);
    addSensor(&JointTargets, string("JointTargets"), JOINT_TARGETS);
    addSensor(&JointStiffnesses, string("JointStiffnesses"), JOINT_STIFFNESSES);
    addSensor(&JointCurrents, string("JointCurrents"), JOINT_CURRENTS);
    addSensor(&JointTorques, string("JointTorques"), JOINT_TORQUES);
    addSensor(&JointTemperatures, string("JointTemperatures"), JOINT_TEMPERATURES);
    
    // Balance Sensors:
    addSensor(&BalanceAccelerometer, string("BalanceAccelerometer"), BALANCE_ACCELEROMETER);
    addSensor(&BalanceGyro, string("BalanceGyro"), BALANCE_GYRO);
    
    // Distance Sensors:
    addSensor(&DistanceValues, string("DistanceValues"), DISTANCE_VALUES);
    
    // Foot Pressure Sensors:
    addSensor(&FootSoleValues, string("FootSoleValues"), FOOT_SOLE_VALUES);
    addSensor(&FootBumperValues, string("FootBumperValues"), FOOT_BUMPER_VALUES);
    
    // Buttons Sensors:
    addSensor(&ButtonValues, string("ButtonValues"), BUTTON_VALUES);
    
    // Battery Sensors:
    addSensor(&BatteryValues, string("BatteryValues"), BATTERY_VALUES);
}

void NUSensorsData::addSensor(sensor_t** p_sensor, string sensorname, sensor_id_t sensorid)
{
    *p_sensor = new sensor_t(sensorname, sensorid);
    m_sensors.push_back(*p_sensor);
}

NUSensorsData::~NUSensorsData()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensorsData::~NUSensorsData" << endl;
#endif
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
    @param bodypartid the id of the body part to get the vector of data
    @param data the variable that will be updated to have the vector of data
 */
bool NUSensorsData::getJointsData(sensor_t* p_sensor, bodypart_id_t bodypartid, vector<float>& data)
{
    switch (bodypartid)
    {
        case All:
            data = p_sensor->Data;
            break;
        default:
            return false;                   //!@todo TODO: implement other body parts!
    }
    return true;
}

/******************************************************************************************************************************************
                                                                                                                                Set Methods
 ******************************************************************************************************************************************/

/*! @brief Sets each of the static joint_id_t if the joint is in the list. Also sets id lists for accessing limbs. 
    @param joints a vector of strings where each string is a name of a joint
 */
void NUSensorsData::setAvaliableJoints(const vector<string> joints)
{
    // first convert everything to lower case and remove whitespace and underscores
    vector<string> simplejointnames;
    string namebuffer, currentname, currentletter;
    for (int i=0; i<joints.size(); i++)
    {
        currentname = joints[i];
        // compare each letter to a space and an underscore
        for (int j=0; j<currentname.size(); j++)
        {
            currentletter = currentname.substr(j, 1);
            if (currentletter.compare(string(" ")) != 0 && currentletter.compare(string("_")) != 0)     // if it is neither then add the lower case version
                namebuffer += tolower(currentletter[0]);            
        }
    }
    
    for (int i=0; i<joints.size(); i++) 
    {
        if (joints[i].compare("headyaw"))
        {
            HeadYaw = i;
            m_head_ids.push_back(i);
        }
        else if (joints[i].compare("headpitch"))
        {
            HeadPitch = i;
            m_head_ids.push_back(i);
        }
        else if (joints[i].compare("lshoulderpitch"))
        {
            LShoulderPitch = i;
            m_larm_ids.push_back(i);
        }
        else if (joints[i].compare("lshoulderroll"))
        {
            LShoulderRoll = i;
            m_larm_ids.push_back(i);
        }
        else if (joints[i].compare("lelbowyaw"))
        {
            LElbowYaw = i;
            m_larm_ids.push_back(i);
        }
        else if (joints[i].compare("lelbowroll"))
        {
            LElbowRoll = i;
            m_larm_ids.push_back(i);
        }
        else if (joints[i].compare("rshoulderpitch"))
        {
            RShoulderPitch = i;
            m_rarm_ids.push_back(i);
        }
        else if (joints[i].compare("rshoulderroll"))
        {
            RShoulderRoll = i;
            m_rarm_ids.push_back(i);
        }
        else if (joints[i].compare("relbowyaw"))
        {
            RElbowYaw = i;
            m_rarm_ids.push_back(i);
        }
        else if (joints[i].compare("relbowroll"))
        {
            RElbowRoll = i;
            m_rarm_ids.push_back(i);
        }
        else if (joints[i].compare("lhipyawpitch"))
        {
            LHipYawPitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (joints[i].compare("lhippitch"))
        {
            LHipPitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (joints[i].compare("lhiproll"))
        {
            LHipRoll = i;
            m_lleg_ids.push_back(i);
        }
        else if (joints[i].compare("lkneepitch"))
        {
            LKneePitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (joints[i].compare("lanklepitch"))
        {
            LAnklePitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (joints[i].compare("lankleroll"))
        {
            LAnkleRoll = i;
            m_lleg_ids.push_back(i);
        }
        else if (joints[i].compare("rhipyawpitch"))
        {
            RHipYawPitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (joints[i].compare("rhippitch"))
        {
            RHipPitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (joints[i].compare("rhiproll"))
        {
            RHipRoll = i;
            m_lleg_ids.push_back(i);
        }
        else if (joints[i].compare("rkneepitch"))
        {
            RKneePitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (joints[i].compare("ranklepitch"))
        {
            RAnklePitch = i;
            m_lleg_ids.push_back(i);
        }
        else if (joints[i].compare("rankleroll"))
        {
            RAnkleRoll = i;
            m_lleg_ids.push_back(i);
        }
    }
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

/*! @brief Sets the distance values to the given values
    @param time the time the data was collected in milliseconds
    @param data the new distance values
    @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setDistanceValues(double time, const vector<float>& data, bool iscalculated)
{
    setData(DistanceValues, time, data, iscalculated);
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

/* @brief Sets the data of the given sensor to the given data
   @param p_sensor a pointer to the sensor to be updated
   @param time the time the data was collected
   @param data the new data to be stored in p_sensor
   @param iscalculated set this to true if the data has been calculated, false otherwise
 */
void NUSensorsData::setData(sensor_t* p_sensor, double time, const vector<float>& data, bool iscalculated)
{
    p_sensor->setData(time, data, iscalculated);
}

/******************************************************************************************************************************************
                                                                                                      Displaying Contents and Serialisation
 ******************************************************************************************************************************************/

/*! @brief Puts a user readable summary of the contents of the NUSensorsData class
    @param output the stream in which to put the summary
 */
void NUSensorsData::summaryTo(ostream& output)
{
    for (int i=0; i<m_sensors.size(); i++)
        m_sensors[i]->summaryTo(output);
}

/*! @todo Implement this function
 */
void NUSensorsData::csvTo(ostream& output)
{
    //! @todo TODO: implement this somewhere somehow!
}

/*! Returns the number of sensors in the NUSensorsData
 */
int NUSensorsData::size() const
{
    return m_sensors.size();
}

ostream& operator<< (ostream& output, const NUSensorsData& p_data)
{
    output << p_data.size() << " ";
    for (int i=0; i<p_data.size(); i++)
        output << *p_data.m_sensors[i];
    return output;
}

istream& operator>> (istream& input, NUSensorsData& p_data)
{
    p_data.m_sensors.clear();
    int numsensors;
    sensor_t insensor;
    sensor_t* sensor;
    input >> numsensors;
    for (int i=0; i<numsensors; i++)
    {
        input >> insensor;
        sensor = new sensor_t(insensor);
        p_data.m_sensors.push_back(sensor);
        p_data.updateNamedSensorPointer(sensor);
    }
    return input;
}

void NUSensorsData::updateNamedSensorPointer(sensor_t* p_sensor)
{
    switch (p_sensor->SensorID) 
    {
        case JOINT_POSITIONS:
            JointPositions = p_sensor;
            break;
        case JOINT_VELOCITIES:
            JointVelocities = p_sensor;
            break;
        case JOINT_ACCELERATIONS:
            JointAccelerations = p_sensor;
            break;
        case JOINT_TARGETS:
            JointTargets = p_sensor;
            break;
        case JOINT_STIFFNESSES:
            JointStiffnesses = p_sensor;
            break;
        case JOINT_CURRENTS:
            JointCurrents = p_sensor;
            break;
        case JOINT_TORQUES:
            JointTorques = p_sensor;
            break;
        case JOINT_TEMPERATURES:
            JointTemperatures = p_sensor;
            break;
        case BALANCE_ACCELEROMETER:
            BalanceAccelerometer = p_sensor;
            break;
        case BALANCE_GYRO:
            BalanceGyro = p_sensor;
            break;
        case DISTANCE_VALUES:
            DistanceValues = p_sensor;
            break;
        case FOOT_SOLE_VALUES:
            FootSoleValues = p_sensor;
            break;
        case FOOT_BUMPER_VALUES:
            FootBumperValues = p_sensor;
            break;
        case BUTTON_VALUES:
            ButtonValues = p_sensor;
            break;
        case BATTERY_VALUES:
            BatteryValues = p_sensor;
            break;
        default:
            break;
    }

}


