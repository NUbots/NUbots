/*! @file NUSensorsData.h
    @brief Declaration of a sensor class to store sensor data in a platform independent way
    @author Jason Kulk
 
    @class NUSensorsData
    @brief A sensor class to store sensor data in a platform independent way
 
    I see a problem coming that I have known about for ages. How am I going to access the data? 
    I need to know the order of the data. I also need to be able to handle requests for joints 
    which aren't there, or are broken!
    
        
        if (NUSensorsData::HeadYaw != NUSensorsData::Unavaliable && m_data->JointPositions->IsValid)
            angle = m_data->JointPositions->Data[NUSensorsData::HeadYaw];
        else
            angle = NaN;
 
        angle = getJointPosition(NUSensorsData::HeadYaw)
        angles = getJointPositions(NUSensorsData::All)          // effectively this is an Aldebaran type approach except instead of strings i'll have an enum
        
 
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

#include "sensor_t.h"

#include <vector>
#include <string>
using namespace std;

class NUSensorsData
{
public:
    NUSensorsData();
    ~NUSensorsData();
    
    void setJointPositions(double time, const vector<float>& data, bool iscalculated = false);
    void setJointVelocities(double time, const vector<float>& data, bool iscalculated = false);
    void setJointAccelerations(double time, const vector<float>& data, bool iscalculated = false);
    void setJointTargets(double time, const vector<float>& data, bool iscalculated = false);
    void setJointStiffnesses(double time, const vector<float>& data, bool iscalculated = false);
    void setJointCurrents(double time, const vector<float>& data, bool iscalculated = false);
    void setJointTorques(double time, const vector<float>& data, bool iscalculated = false);
    void setJointTemperatures(double time, const vector<float>& data, bool iscalculated = false);
    void setBalanceAccelerometer(double time, const vector<float>& data, bool iscalculated = false);
    void setBalanceGyro(double time, const vector<float>& data, bool iscalculated = false);
    void setDistanceValues(double time, const vector<float>& data, bool iscalculated = false);
    void setFootSoleValues(double time, const vector<float>& data, bool iscalculated = false);
    void setFootBumperValues(double time, const vector<float>& data, bool iscalculated = false);
    void setButtonValues(double time, const vector<float>& data, bool iscalculated = false);
    void setBatteryValues(double time, const vector<float>& data, bool iscalculated = false);
    
    void summaryTo(ostream& output);
    void csvTo(ostream& output);
    
    friend ostream& operator<< (ostream& output, const NUSensorsData& p_sensor);
    friend istream& operator>> (istream& input, NUSensorsData& p_sensor);
    
    int size() const;
private:
    void addSensor(sensor_t** p_sensor, string sensorname, sensor_id_t sensorid);
    
    void setData(sensor_t* p_sensor, double time, const vector<float>& data, bool iscalculated = false);
    
    void updateNamedSensorPointer(sensor_t* p_sensor);
public:
    // NAMED SENSORS
    // Proprioception Sensors:
    sensor_t* JointPositions;
    sensor_t* JointVelocities;
    sensor_t* JointAccelerations;
    sensor_t* JointTargets;
    sensor_t* JointStiffnesses;
    sensor_t* JointCurrents;
    sensor_t* JointTorques;
    sensor_t* JointTemperatures;
    
    // Balance Sensors:
    sensor_t* BalanceAccelerometer;             //!< stores the sensor measurements for the linear acceleration of the torso in cm/s/s
    sensor_t* BalanceGyro;                      //!< stores the sensor measurements for the radial velocities of the torso in rad/s
    
    // Distance Sensors:
    sensor_t* DistanceValues;
    
    // Foot Pressure Sensors:
    sensor_t* FootSoleValues;
    sensor_t* FootBumperValues;
    
    // Buttons Sensors:
    sensor_t* ButtonValues;
    
    // Battery Sensors:
    sensor_t* BatteryValues;
    
private:
    vector<sensor_t*> m_sensors;
};

#endif

