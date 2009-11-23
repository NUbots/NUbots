/*! @file sensor_t.h
    @brief Declaration of a single set of sensor class
    @author Jason Kulk
 
    @class sensor_t
    @brief A single set of sensors class to store sensor data in a platform independent way
 
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

#ifndef SENSOR_T_H
#define SENSOR_T_H

#include <vector>
#include <string>
using namespace std;

enum sensor_id_t 
{
    JOINT_POSITIONS,
    UNDEFINED
};


class sensor_t 
{
public:
    sensor_t()
    {
        Name = string("Undefined");
        SensorID = UNDEFINED;
        IsValid = false;
        IsCalculated = false;
        TimeStamp = 0;
    };
    sensor_t(string sensorname, sensor_id_t sensorid)
    {
        Name = sensorname; 
        SensorID = sensorid;
        IsValid = false;
        IsCalculated = false;
        TimeStamp = 0;
    };
public:
    string Name;                //!< the sensor's name
    sensor_id_t SensorID;       //!< the sensor's id
    vector<float> Data;         //!< the sensor values
    vector<float> StdDev;       //!< standard deviation for each sensor value sensor
    bool IsValid;               //!< true, if data is valid, false if not
    bool IsCalculated;          //!< true, if data has been calculated, false if the data is direct from a sensor
    long double TimeStamp;      //!< the unix timestamp of the data
};

#endif

