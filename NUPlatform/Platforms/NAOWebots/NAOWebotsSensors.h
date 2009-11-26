/*! @file NAOWebotsSensors.h
    @brief Declaration of NAO in Webots sensors class

    @author Jason Kulk
 
    @class NAOWebotsSensors
    @brief A NAO in Webots sensors
 
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

#ifndef NAOWEBOTSSENSORS_H
#define NAOWEBOTSSENSORS_H

#include "NUPlatform/NUSensors.h"
#include "NAOWebotsPlatform.h"

#include <vector>
#include <string>
#include <fstream>
using namespace std;

#include "webots/Robot.hpp"
using namespace webots;


class NAOWebotsSensors : public NUSensors
{
public:
    NAOWebotsSensors(NAOWebotsPlatform* platform);
    ~NAOWebotsSensors();
    
private:
    void copyFromHardwareCommunications();
    
    void getSensorsFromWebots(NAOWebotsPlatform* platform);
    void enableSensorsInWebots();
    
private:
    static const int m_simulation_step = 40;            //!< the refresh period of the sensor data in milliseconds. Robotstadium's timestep is fixed at 40ms
    
    NAOWebotsPlatform* m_platform;
    // Sensors
    static vector<string> m_servo_names;                //!< a vector of the names of each servo in the Webot NAO
    vector<Servo*> m_servos;                            //!< a vector containing pointers to each of the servos in the Webot NAO.
    Accelerometer* m_accelerometer;                     //!< a pointer to the robot's accelerometer
    Gyro* m_gyro;                                       //!< a pointer to the robot's gyrometer
    static vector<string> m_distance_names;             //!< a vector of the names of each of the distance sensors in Webot NAO
    vector<DistanceSensor*> m_distance_sensors;         //!< a vector containing pointers to each of the distance sensors in the Webot NAO
    static vector<string> m_foot_sole_names;            //!< a vector of the names of each of the foot touch sensors in Webot NAO
    vector<TouchSensor*> m_foot_sole_sensors;           //!< a vector of pointers to each of the foot force sensors in the Webot NAO
    static vector<string> m_foot_bumper_names;          //!< a vector of the foot bumper names
    vector<TouchSensor*> m_foot_bumper_sensors;         //!< a vector of pointers to buttons
    GPS* m_gps;                                         //!< a pointer to the gps module of the robot available for testing!
    
    ifstream tempinlog;             // temp log for testing serialisation
    ofstream tempoutlog;            // temp log for testing serialisation
    NUSensorsData* m_temp_data;     // a version of NUSensorsData that has been loaded from a log
};

#endif

