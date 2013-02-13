/*! @file Sensor.cpp
    @brief Implementation of a single set of sensor class
    @author Jason Kulk
 
  Copyright (c) 2010 Jason Kulk
 
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
    Time = 0.0;
    ValidFloat = false;
    ValidVector = false;
    ValidMatrix = false;
    ValidString = false;
    
    VectorData.reserve(128);
    MatrixData.reserve(64);
}

/*! @brief Copy constructor for a Sensor
    @param source The source of the copy
 */
Sensor::Sensor(const Sensor& source)
{
    Name = source.Name;
    Time = source.Time;
    ValidFloat = source.ValidFloat;
    ValidVector = source.ValidVector;
    ValidMatrix = source.ValidMatrix;
    ValidString = source.ValidString;

    FloatData = source.FloatData;
    VectorData = source.VectorData;
    MatrixData = source.MatrixData;
    StringData = source.StringData;
}

Sensor& Sensor::operator= (const Sensor & source)
{
    Name = source.Name;
    Time = source.Time;
    ValidFloat = source.ValidFloat;
    ValidVector = source.ValidVector;
    ValidMatrix = source.ValidMatrix;
    ValidString = source.ValidString;

    FloatData = source.FloatData;
    VectorData = source.VectorData;
    MatrixData = source.MatrixData;
    StringData = source.StringData;

    return *this;
}

/*! @brief Gets float sensor reading, returns true if sucessful, false otherwise 
    @param data will be updated with reading
    @return true if valid sensor reading, false otherwise
 */
bool Sensor::get(float& data) const
{
    if (ValidFloat)
    {
        data = FloatData;
        return true;
    }
    else
        return false;
}

/*! @brief Gets vector sensor reading, returns true if sucessful, false otherwise 
    @param data will be updated with reading
    @return true if valid sensor reading, false otherwise
 */
bool Sensor::get(vector<float>& data) const
{
    if (ValidVector)
    {
        data = VectorData;
        return true;
    }
    else
        return false;
}

/*! @brief Gets matrix sensor reading, returns true if sucessful, false otherwise 
    @param data will be updated with reading
    @return true if valid sensor reading, false otherwise
 */
bool Sensor::get(vector<vector<float> >& data) const
{
    if (ValidMatrix)
    {
        data = MatrixData;
        return true;
    }
    else
        return false;
}

/*! @brief Gets string sensor reading, returns true if sucessful, false otherwise 
    @param data will be updated with reading
    @return true if valid sensor reading, false otherwise
 */
bool Sensor::get(string& data) const
{
    if (ValidString)
    {
        data = StringData;
        return true;
    }
    else
        return false;
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

/*! @brief Sets all of the sensor data as being invalid */
void Sensor::setAsInvalid()
{
    ValidFloat = false;
    ValidVector = false;
    ValidMatrix = false;
    ValidString = false;
}

/*! @brief Modify existing vector sensor data. This is especially for packed sensors which are share the same Sensor instance.
 	@param time the new sensor data time in ms
 	@param start the position in which the new data will be inserted 
 	@param data the data to insert 
 */
void Sensor::modify(double time, unsigned int start, const float& data)
{
    Time = time;
    if (ValidVector)
    {
        size_t s = VectorData.size();
        if (start < s)
        	VectorData[start] = data;
        else if (s == start)
            VectorData.push_back(data);
    }
    else if (ValidFloat)
    {
        ValidFloat = false;
        ValidVector = true;
        VectorData = vector<float>(2, FloatData);
        VectorData[1] = data;
    }
    else if (start == 0)
    {
        ValidVector = true;
        VectorData = vector<float>(1, data);
	}
}

/*! @brief Modify existing vector sensor data. This is especially for packed sensors which are share the same Sensor instance.
 	@param time the new sensor data time in ms
 	@param start the first position in which the new data will be inserted 
 	@param data the data to insert 
 */
void Sensor::modify(double time, unsigned int start, const vector<float>& data)
{
    Time = time;
    if (ValidVector)
    {
        for (size_t i=0; i<data.size(); i++)
        {
            size_t s = VectorData.size();
            if (start+i < s)
                VectorData[start+i] = data[i];
            else if (s == start+i)
                VectorData.push_back(data[i]);
        }
    }
    else if (start == 0)
    {
        ValidVector = true;
        VectorData = data;
	}
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


