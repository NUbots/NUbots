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
                                                                                                                                Set Methods
 ******************************************************************************************************************************************/

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
    // TODO: implement this somewhere somehow!
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


