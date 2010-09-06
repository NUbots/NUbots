/*! @file Sensor.cpp
    @brief Implementation of a single set of sensor class
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

#include "Sensor.h"

#include "Tools/Math/StlVector.h"

#include "debug.h"
#include "debugverbositynusensors.h"
    
/*! @brief Constructor for a Sensor
    @param sensorname the name of the sensor
 */
Sensor::Sensor(string sensorname)
{
    Name = sensorname; 
    ValidFloat = false;
    ValidVector = false;
    ValidMatrix = false;
    ValidString = false;
}

/*! @brief Updates the sensors data
    @param time the time in milliseconds the data was captured
    @param data the new sensor data
 */
void Sensor::set(double time, const float& data)
{
    Time = time;
    FloatData = data;
    ValidFloat = true;
    ValidVector = false;
    ValidMatrix = false;
    ValidString = false;
}

/*! @brief Updates the sensors data
    @param time the time in milliseconds the data was captured
    @param data the vector of new sensor data
 */
void Sensor::set(double time, const vector<float>& data)
{
    Time = time;
    VectorData = data;
    ValidVector = true;
    ValidFloat = false;
    ValidMatrix = false;
    ValidString = false;
}

/*! @brief Updates the sensors data
    @param time the time in milliseconds the data was captured
    @param data the matrix of new sensor data
 */
void Sensor::set(double time, const vector<vector<float> >& data)
{
    Time = time;
    MatrixData = data;
    ValidMatrix = true;
    ValidFloat = false;
    ValidVector = false;
    ValidString = false;
}

/*! @brief Updates the sensors data
    @param time the time in milliseconds the data was captured
    @param data the string of new sensor data
 */
void Sensor::set(double time, const string& data)
{
    Time = time;
    StringData = data;
    ValidString = true;
    ValidFloat = false;
    ValidVector = false;
    ValidMatrix = false;
}

/*! @brief Provides a text summary of the contents of the Sensor
 
    The idea is to use this function when writing to a debug log. I guarentee that the 
    output will be human readable.
 
    @param output the ostream in which to put the string
 */
void Sensor::summaryTo(ostream& output) const
{
    if (ValidFloat or ValidVector or ValidMatrix or ValidString)
    {
        output << Name << ": " << Time << " ";
        if (ValidFloat)
            output << FloatData;
        else if (ValidVector)
            output << VectorData;
        else if (ValidMatrix)
            output << MatrixData;
        else if (ValidString)
            output << "\"" << StringData << "\"";
        output << endl;
    }
}

/*! @brief Saves the entire contents of the Sensor into the stream
 
    The data in the stream will not be human readable as some of the data
    is written in binary mode.
 */
ostream& operator<< (ostream& output, const Sensor& p_sensor)
{
    output << p_sensor.Name << " ";
    output << p_sensor.Time << " ";
    output << p_sensor.ValidFloat << " " << p_sensor.ValidVector << " " << p_sensor.ValidMatrix << " " << p_sensor.ValidString << " ";
    if (p_sensor.ValidFloat)
        output << p_sensor.FloatData;
    else if (p_sensor.ValidVector)
        output << p_sensor.VectorData;
    else if (p_sensor.ValidMatrix)
        output << p_sensor.MatrixData;
    else if (p_sensor.ValidString)
        output << p_sensor.StringData;
    output << endl;
    return output;
}

/*! @brief Loads the entire contents of the Sensor into the stream
 
     The data in the stream will not be human readable as some of the data
     is written in binary mode.
 */
istream& operator>> (istream& input, Sensor& p_sensor)
{
    input >> p_sensor.Name;
    input >> p_sensor.Time;
    input >> p_sensor.ValidFloat >> p_sensor.ValidVector >> p_sensor.ValidMatrix >> p_sensor.ValidString;
    if (p_sensor.ValidFloat)
        input >> p_sensor.FloatData;
    else if (p_sensor.ValidVector)
        input >> p_sensor.VectorData;
    else if (p_sensor.ValidMatrix)
        input >> p_sensor.MatrixData;
    else if (p_sensor.ValidString)
        input >> p_sensor.StringData;
    return input;
}


