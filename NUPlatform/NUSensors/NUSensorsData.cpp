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

void NUSensorsData::summaryTo(ostream& output)
{
    for (int i=0; i<m_sensors.size(); i++)
        m_sensors[i]->summaryTo(output);
}

void NUSensorsData::csvTo(ostream& output)
{
    // TODO: implement this somewhere!
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


