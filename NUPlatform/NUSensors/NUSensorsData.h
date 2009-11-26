/*! @file NUSensorsData.h
    @brief Declaration of a sensor class to store sensor data in a platform independent way
    @author Jason Kulk
 
    @class NUSensorsData
    @brief A sensor class to store sensor data in a platform independent way
 
    I see a problem coming that I have known about for ages. How am I going to access the data? 
    I need to know the order of the data. I also need to be able to handle requests for joints 
    which aren't there, or are broken!
    
 
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

typedef int joint_id_t;

class NUSensorsData
{
public:
    static const int SENSOR_MISSING = -1;
    enum bodypart_id_t
    {
        Head,
        LeftArm,
        RightArm,
        Torso,
        LeftLeg,
        RightLeg,
        All
    };
    static joint_id_t HeadYaw;
    static joint_id_t HeadPitch;
    static joint_id_t LShoulderPitch;
    static joint_id_t LShoulderRoll;
    static joint_id_t LElbowYaw;
    static joint_id_t LElbowRoll;
    static joint_id_t RShoulderPitch;
    static joint_id_t RShoulderRoll;
    static joint_id_t RElbowYaw;
    static joint_id_t RElbowRoll;
    static joint_id_t LHipYawPitch;
    static joint_id_t LHipPitch;
    static joint_id_t LHipRoll;
    static joint_id_t LKneePitch;
    static joint_id_t LAnklePitch;
    static joint_id_t LAnkleRoll;
    static joint_id_t RHipYawPitch;
    static joint_id_t RHipPitch;
    static joint_id_t RHipRoll;
    static joint_id_t RKneePitch;
    static joint_id_t RAnklePitch;
    static joint_id_t RAnkleRoll;
public:
    NUSensorsData();
    ~NUSensorsData();
    
    bool getJointPosition(joint_id_t jointid, float& position);
    bool getJointVelocity(joint_id_t jointid, float& velocity);
    bool getJointAcceleration(joint_id_t jointid, float& acceleration);
    bool getJointTarget(joint_id_t jointid, float& target);
    bool getJointStiffness(joint_id_t jointid, float& stiffness);
    bool getJointCurrent(joint_id_t jointid, float& current);
    bool getJointTorque(joint_id_t jointid, float& torque);
    bool getJointTemperature(joint_id_t jointid, float& temperature);
    
    bool getJointPositions(bodypart_id_t bodypart, vector<float>& positions);
    bool getJointVelocities(bodypart_id_t bodypart, vector<float>& velocities);
    bool getJointAccelerations(bodypart_id_t bodypart, vector<float>& accelerations);
    bool getJointTargets(bodypart_id_t bodypart, vector<float>& targets);
    bool getJointStiffnesses(bodypart_id_t bodypart, vector<float>& stiffnesses);
    bool getJointCurrents(bodypart_id_t bodypart, vector<float>& currents);
    bool getJointTorques(bodypart_id_t bodypart, vector<float>& torques);
    bool getJointTemperatures(bodypart_id_t bodypart, vector<float>& temperatures);
    
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
    
    bool getJointData(sensor_t* p_sensor, joint_id_t jointid, float& data);
    bool getJointsData(sensor_t* p_sensor, bodypart_id_t bodypartid, vector<float>& data);
    
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

