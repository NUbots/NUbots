/*! @file sensor_t.h
    @brief Declaration of a single set of sensor class
    @author Jason Kulk
 
    @class sensor_t
    @brief A single set of sensors class to store sensor data in a platform independent way
 
    A sensor_t is a container for a set of similar sensors that share a common timestamp.
    For example, all of the JoinPositions are encapsulated in a single sensor_t.
 
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

/*! @brief A enum type to perform run time sensor type checking without using string
           string compares. Unfortunately, to add a new sensor you need to add an
           id to this list.
 */
enum sensor_id_t 
{
    JOINT_POSITIONS,
    JOINT_VELOCITIES,
    JOINT_ACCELERATIONS,
    JOINT_TARGETS,
    JOINT_STIFFNESSES,
    JOINT_CURRENTS,
    JOINT_TORQUES,
    JOINT_TEMPERATURES,
    BALANCE_VALUES,
    DISTANCE_VALUES,
    FOOT_SOLE_VALUES,
    FOOT_BUMPER_VALUES,
    BUTTON_VALUES,
    BATTERY_VALUES,
    UNDEFINED
};


class sensor_t 
{
public:
    sensor_t();
    sensor_t(string sensorname, sensor_id_t sensorid);
    void setData(double time, vector<float> newdata, bool iscalculated = false);
    void setStdDev(vector<float> newstddev);
    
    void summaryTo(ostream& output);
    void csvTo(ostream& output);
    
    friend ostream& operator<< (ostream& output, const sensor_t& p_sensor);
    friend istream& operator>> (istream& input, sensor_t& p_sensor);
public:
    string Name;                //!< the sensor's name
    sensor_id_t SensorID;       //!< the sensor's id
    vector<float> Data;         //!< the sensor values
    vector<float> StdDev;       //!< standard deviation for each sensor value sensor
    bool IsValid;               //!< true, if data is valid, false if not
    bool IsCalculated;          //!< true, if data has been calculated, false if the data is direct from a sensor
    double Time;                //!< the sensor's time
    long double TimeStamp;      //!< the unix timestamp of the data
private:
    long double m_time_offset;       //!< a time offset so that unixtimestamp = timesincestart + offset
};

#endif

