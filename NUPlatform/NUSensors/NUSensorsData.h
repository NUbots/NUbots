/*! @file NUSensorsData.h
    @brief Declaration of a sensor class to store sensor data in a platform independent way
    @author Jason Kulk
 
    @class NUSensorsData
    @brief A sensor class to store sensor data in a platform independent way
 
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

#ifndef NUSENSORSDATA_H
#define NUSENSORSDATA_H

#include <vector>
#include <string>
using namespace std;

/*! @brief Container for a single group of sensors
 */
struct sensor_t 
{
    string name;                //!< the sensor's name
    int sensorID;               //!< the sensor's id
    vector<float> data;         //!< the sensor values
    vector<float> sd;           //!< standard deviation for each sensor value sensor
    bool isValid;               //!< true, if data is valid, false if not
    bool isCalculated;          //!< true, if data has been calculated, false if the data is direct from a sensor
    long double timestamp;      //!< the unix timestamp of the data
};

/*! @brief Base sensor storage class
 */
class NUSensorsData
{
public:
    NUSensorsData();
    ~NUSensorsData();
    
private:
public:
    
    
private:
    vector<sensor_t*> data;
    // Proprioception Sensors:
    sensor_t m_joint_positions;
    sensor_t m_joint_velocities;
    sensor_t m_joint_accelerations;
    sensor_t m_joint_targets;
    sensor_t m_joint_stiffnesses;
    sensor_t m_joint_currents;
    sensor_t m_joint_temperatures;
    
    // Balance Sensors:
    sensor_t m_balance_values;
    
    // Touch Sensors:
    sensor_t m_touch_values;
    
    // Foot Pressure Sensors:
    sensor_t m_foot_values;
    
    // Battery Sensors:
    sensor_t m_battery_values;
    
    // Distance Sensors:
    sensor_t m_distance_values;
};

#endif

