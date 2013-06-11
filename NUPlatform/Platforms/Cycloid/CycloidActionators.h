/*! @file Sensor.h
    @brief Declaration of a single sensor class
    @author Jason Kulk
 
    @class Sensor
    @brief A single set of sensors class to store sensor data in a platform independent way
 
    A Sensor is a container for a set of similar sensors that share a common time.
    For example, all of the JointPositions are encapsulated in a single Sensor.
 
    @author Jason Kulk
 
  Copyright (c) 2009, 2010 Jason Kulk
 
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

#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include <string>


class Sensor 
{
public:
    Sensor(std::string sensorname);
    Sensor(const Sensor& source);

    bool get(float& data) const;
    bool get(std::vector<float>& data) const;
    bool get(std::vector<std::vector<float> >& data) const;
    bool get(std::string& data) const;
    
    void set(double time, const float& data);
    void set(double time, const std::vector<float>& data);
    void set(double time, const std::vector<std::vector<float> >& data);
    void set(double time, const std::string& data);
    void setAsInvalid();
    
    void modify(double time, unsigned int start, const float& data);
    void modify(double time, unsigned int start, const std::vector<float>& data);
    
    void summaryTo(std::ostream& output) const;
    
    friend std::ostream& operator<< (std::ostream& output, const Sensor& p_sensor);
    friend std::istream& operator>> (std::istream& input, Sensor& p_sensor);
    Sensor& operator= (const Sensor & source);
public:
    std::string Name;                        //!< the sensor's name
    double Time;                        //!< the timestamp associated with the data
private:
    // only a single type of data is used at one time
    // the Valid flags are used to tell which type is the valid one
    float FloatData;                    //!< the float data
    bool ValidFloat;                    //!< a flag to indicate whether the float data is valid
    std::vector<float> VectorData;           //!< the std::vector data
    bool ValidVector;                   //!< a flag to indicate whether the std::vector data is valid
    std::vector<std::vector<float> > MatrixData;  //!< the matrix data
    bool ValidMatrix;                   //!< a flag to indicate whether the matrix data is valid
    std::string StringData;                  //!< the std::string data
    bool ValidString;                   //!< a flag to indicate whether the std::string data is valid
};

#endif

