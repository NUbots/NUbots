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
    JointPositions = new sensor_t(string("JointPositions"), JOINT_POSITIONS);
    JointVelocities = new sensor_t(string("JointVelocities"), JOINT_VELOCITIES);
    JointAccelerations = new sensor_t(string("JointAccelerations"), JOINT_ACCELERATIONS);
    JointTargets = new sensor_t(string("JointTargets"), JOINT_TARGETS);
    JointStiffnesses = new sensor_t(string("JointStiffnesses"), JOINT_STIFFNESSES);
    JointCurrents = new sensor_t(string("JointCurrents"), JOINT_CURRENTS);
    JointTorques = new sensor_t(string("JointTorques"), JOINT_TORQUES);
    JointTemperatures = new sensor_t(string("JointTemperatures"), JOINT_TEMPERATURES);
    
    // Balance Sensors:
    BalanceValues = new sensor_t(string("BalanceValues"), BALANCE_VALUES);
    
    // Distance Sensors:
    DistanceValues = new sensor_t(string("DistanceValues"), DISTANCE_VALUES);
    
    // Foot Pressure Sensors:
    FootSoleValues = new sensor_t(string("FootSoleValues"), FOOT_SOLE_VALUES);
    FootBumperValues = new sensor_t(string("FootBumperValues"), FOOT_BUMPER_VALUES);
    
    // Buttons Sensors:
    ButtonValues = new sensor_t(string("ButtonValues"), BUTTON_VALUES);
    
    // Battery Sensors:
    BatteryValues = new sensor_t(string("BatteryValues"), BATTERY_VALUES);
}

NUSensorsData::~NUSensorsData()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NUSensorsData::~NUSensorsData" << endl;
#endif
}


