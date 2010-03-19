/*! @file sensor_t.cpp
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

#include "sensor_t.h"
#include "NUPlatform/NUSystem.h"
#include "debug.h"


/*! @brief Default constructor for a sensor_t. Initialises the sensor to be undefined and invalid
 */
sensor_t::sensor_t()
{
    Name = string("Undefined");
    SensorID = UNDEFINED;
    IsValid = false;
    IsCalculated = false;
    TimeStamp = 0;
}
    
/*! @brief Constructor for a named sensor_t
    @param sensorname the name of the sensor
    @param sensorid the id of the sensor
 */
sensor_t::sensor_t(string sensorname, sensor_id_t sensorid, bool iscalculated)
{
    Name = sensorname; 
    SensorID = sensorid;
    IsValid = false;
    IsCalculated = iscalculated;
    m_time_offset = NUSystem::getTimeOffset();
    Time = 0;
    TimeStamp = Time + m_time_offset;
}

/*! @brief Updates the sensors data
    @param time the time in milliseconds since the program started
    @param newdata the vector of new sensor data
    @param iscalculated set this to true if the new sensor data was calculated from other sensor data
 */
void sensor_t::setData(double time, vector<float> newdata, bool iscalculated)
{
    Time = time;
    TimeStamp = Time + m_time_offset;
    Data = newdata;
    IsValid = true;
    IsCalculated = iscalculated;
}

/*! @brief Sets/updates the sensor standard deviation
    @param newstddev the vector of standard deviations. It is assumed that the order of newstddevs matches the newdata
 */
void sensor_t::setStdDev(vector<float> newstddev)
{
    StdDev = newstddev;
}

/*! @brief Provides a text summary of the contents of the sensor_t
 
    The idea is to use this function when writing to a debug log. I guarentee that the 
    output will be human readable.
 
    @param output the ostream in which to put the string
 */
void sensor_t::summaryTo(ostream& output)
{
    output << Name << ": " << Time << " ";
    if (IsValid)
    {
        for (unsigned int i=0; i<Data.size(); i++)
            output << Data[i] << " ";
    }
    else
        output << "Invalid";
    output << endl;
}

/*! @brief Provides a comma separated string of data for writing to a .csv
    @param output the ostream in which to put the string
 */
void sensor_t::csvTo(ostream& output)
{
    output << Time << ", ";
    if (IsValid)
    {
        for (unsigned int i=0; i<Data.size(); i++)
            output << Data[i] << ", ";
    }
    output << endl;
}



/*! @brief Saves the entire contents of the sensor_t into the stream
 
    The data in the stream will not be human readable as some of the data
    is written in binary mode.
 */
ostream& operator<< (ostream& output, const sensor_t& p_sensor)
{
    output << p_sensor.Name << " ";
    output << (int) p_sensor.SensorID << " ";
    
    output << p_sensor.Data.size() << " ";
    // we save the sensor data as binary data
    for (unsigned int i=0; i<p_sensor.Data.size(); i++)
        output.write((char*) &p_sensor.Data[i], sizeof(float));
    output << " ";
    
    output << p_sensor.StdDev.size() << " ";
    // we save the standard deviation data as binary data
    for (unsigned int i=0; i<p_sensor.StdDev.size(); i++)
        output.write((char*) &p_sensor.StdDev[i], sizeof(float));
    output << " ";
    
    output << p_sensor.IsValid << " ";
    output << p_sensor.IsCalculated << " ";
    
    // we save the time as binary data
    output.write((char*) &p_sensor.Time, sizeof(double));
    // we also save the timestamp as binary data
    output.write((char*) &p_sensor.TimeStamp, sizeof(long double));
    return output;
}

/*! @brief Loads the entire contents of the sensor_t into the stream
 
     The data in the stream will not be human readable as some of the data
     is written in binary mode.
 */
istream& operator>> (istream& input, sensor_t& p_sensor)
{
    int size = 0;
    int id = 9000;

    // temporary buffers for binary read
    char charBuffer;
    float floatBuffer;
    double doubleBuffer;
    long double lDoubleBuffer;

    input >> p_sensor.Name;
    input >> id;
    p_sensor.SensorID = (sensor_t::sensor_id_t) id;
    
    input >> size;
    p_sensor.Data.resize(size, 0);
    input.read(&charBuffer, sizeof(char));         // skip over the single space after the size
    for (int i=0; i<size; i++)
    {
        input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(float));
        p_sensor.Data[i] = floatBuffer;
    }
    
    input >> size;
    p_sensor.StdDev.resize(size, 0);
    input.read(&charBuffer, sizeof(char));         // skip over the single space after the size
    for (int i=0; i<size; i++)
    {
        input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(float));
        p_sensor.StdDev[i] = floatBuffer;
    }
    
    // Read in flags
    input >> p_sensor.IsValid;
    input >> p_sensor.IsCalculated;
    // Read in Time
    input.read(&charBuffer, sizeof(char));         // skip over the single space after the iscalculated flag
    input.read(reinterpret_cast<char*>(&doubleBuffer), sizeof(double));
    p_sensor.Time = doubleBuffer;
    // Read in TimeStamp
    input.read(reinterpret_cast<char*>(&lDoubleBuffer), sizeof(long double));
    p_sensor.TimeStamp = lDoubleBuffer;
    p_sensor.m_time_offset = p_sensor.TimeStamp - p_sensor.Time;
    return input;
}

/*! @brief Overloaded subscript operator has been written for easy access to sensor data.
 */
float& sensor_t::operator[] (const unsigned int index)
{
    if (index < Data.size())
        return Data[index];
    else
        return Data[Data.size() - 1];
}

/*! @brief Returns the size of the sensor_t, where the size is the size of Data
 */
int sensor_t::size()
{
    return Data.size();
}


