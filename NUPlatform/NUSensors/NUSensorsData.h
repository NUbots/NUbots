/*! @file NUSensorsData.h
    @brief Declaration of a sensor data storage class to store sensor data in a platform independent way
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

#include "sensor_t.h"

#include <vector>
#include <string>
using namespace std;

class NUSensorsData
{
public:
    typedef int joint_id_t;
    
    static const int SENSOR_MISSING = -1;
    // joint ids (these are used to index into named sensor_t data)
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
    static joint_id_t LHipYaw;
    static joint_id_t LHipYawPitch;
    static joint_id_t LHipPitch;
    static joint_id_t LHipRoll;
    static joint_id_t LKneePitch;
    static joint_id_t LAnklePitch;
    static joint_id_t LAnkleRoll;
    static joint_id_t RHipYaw;
    static joint_id_t RHipYawPitch;
    static joint_id_t RHipPitch;
    static joint_id_t RHipRoll;
    static joint_id_t RKneePitch;
    static joint_id_t RAnklePitch;
    static joint_id_t RAnkleRoll;
    // limb ids
    enum bodypart_id_t
    {
        HeadJoints,
        LeftArmJoints,
        RightArmJoints,
        TorsoJoints,
        LeftLegJoints,
        RightLegJoints,
        AllJoints
    };
    // foot ids
    enum foot_id_t 
    {
        LeftFoot,
        RightFoot,
        AllFeet
    };
    // button ids
    enum button_id_t
    {
        MainButton,
        SecondaryButton,
        AllButtons
    };
public:
    NUSensorsData();
    ~NUSensorsData();
    
    // Get methods for a single joint at a time
    bool getJointPosition(joint_id_t jointid, float& position);
    bool getJointVelocity(joint_id_t jointid, float& velocity);
    bool getJointAcceleration(joint_id_t jointid, float& acceleration);
    bool getJointTarget(joint_id_t jointid, float& target);
    bool getJointStiffness(joint_id_t jointid, float& stiffness);
    bool getJointCurrent(joint_id_t jointid, float& current);
    bool getJointTorque(joint_id_t jointid, float& torque);
    bool getJointTemperature(joint_id_t jointid, float& temperature);
    
    // Get methods for a limb of joints (the limb can also be body and all)
    bool getJointPositions(bodypart_id_t bodypart, vector<float>& positions);
    bool getJointVelocities(bodypart_id_t bodypart, vector<float>& velocities);
    bool getJointAccelerations(bodypart_id_t bodypart, vector<float>& accelerations);
    bool getJointTargets(bodypart_id_t bodypart, vector<float>& targets);
    bool getJointStiffnesses(bodypart_id_t bodypart, vector<float>& stiffnesses);
    bool getJointCurrents(bodypart_id_t bodypart, vector<float>& currents);
    bool getJointTorques(bodypart_id_t bodypart, vector<float>& torques);
    bool getJointTemperatures(bodypart_id_t bodypart, vector<float>& temperatures);
    
    // Get methods for the other sensors
    bool getAccelerometerValues(vector<float>& values);
    bool getGyroValues(vector<float>& values);
    bool getDistanceValues(vector<float>& values);
    bool getBatteryValues(vector<float>& values);
    bool getGPSValues(vector<float>& values);
    
    // Get methods for other sensors that have logical groups
    bool getFootSoleValues(foot_id_t footid, vector<float>& values);
    bool getFootBumperValues(foot_id_t footid, vector<float>& values);
    bool getButtonValues(button_id_t buttonid, vector<float>& values);
    
    void setAvailableJoints(const vector<string>& joints);
    
    // Set methods for joints
    void setJointPositions(double time, const vector<float>& data, bool iscalculated = false);
    void setJointVelocities(double time, const vector<float>& data, bool iscalculated = false);
    void setJointAccelerations(double time, const vector<float>& data, bool iscalculated = false);
    void setJointTargets(double time, const vector<float>& data, bool iscalculated = false);
    void setJointStiffnesses(double time, const vector<float>& data, bool iscalculated = false);
    void setJointCurrents(double time, const vector<float>& data, bool iscalculated = false);
    void setJointTorques(double time, const vector<float>& data, bool iscalculated = false);
    void setJointTemperatures(double time, const vector<float>& data, bool iscalculated = false);
    
    // Set methods for other sensors
    void setBalanceAccelerometer(double time, const vector<float>& data, bool iscalculated = false);
    void setBalanceGyro(double time, const vector<float>& data, bool iscalculated = false);
    void setDistanceValues(double time, const vector<float>& data, bool iscalculated = false);
    void setFootSoleValues(double time, const vector<float>& data, bool iscalculated = false);
    void setFootBumperValues(double time, const vector<float>& data, bool iscalculated = false);
    void setButtonValues(double time, const vector<float>& data, bool iscalculated = false);
    void setBatteryValues(double time, const vector<float>& data, bool iscalculated = false);
    void setGPSValues(double time, const vector<float>& data, bool iscalculated = false);
    
    void summaryTo(ostream& output);
    void csvTo(ostream& output);
    
    friend ostream& operator<< (ostream& output, const NUSensorsData& p_sensor);
    friend istream& operator>> (istream& input, NUSensorsData& p_sensor);
    
    int size() const;
private:
    void addSensor(sensor_t*& p_sensor, string sensorname, sensor_t::sensor_id_t sensorid);
    
    bool getJointData(sensor_t* p_sensor, joint_id_t jointid, float& data);
    bool getJointsData(sensor_t* p_sensor, bodypart_id_t bodypartid, vector<float>& data);
    
    void setData(sensor_t* p_sensor, double time, const vector<float>& data, bool iscalculated = false);
    
    void updateNamedSensorPointer(sensor_t* p_sensor);
public:
    // NAMED SENSORS
    // Proprioception Sensors:
    sensor_t* JointPositions;                   //!< stores the joint position sensors (in radians)
    sensor_t* JointVelocities;                  //!< stores the joint velocity sensors (in rad/s)
    sensor_t* JointAccelerations;               //!< stores the joint acceleration sensors (in rad/s/s)
    sensor_t* JointTargets;                     //!< stores the joint position targets (in radians)
    sensor_t* JointStiffnesses;                 //!< stores the joint stiffness values (as a percent)
    sensor_t* JointCurrents;                    //!< stores the joint motor current sensors (in A)
    sensor_t* JointTorques;                     //!< stores the joint temperatures (in degrees C)
    sensor_t* JointTemperatures;
    
    // Balance Sensors:
    sensor_t* BalanceAccelerometer;             //!< stores the sensor measurements for the linear acceleration of the torso in cm/s/s
    sensor_t* BalanceGyro;                      //!< stores the sensor measurements for the radial velocities of the torso in rad/s
    
    // Distance Sensors:
    sensor_t* DistanceValues;                   //!< stores the distance to obstacle measurements in cm
    
    // Foot Pressure Sensors:
    sensor_t* FootSoleValues;                   //!< stores the foot force in Newtons
    sensor_t* FootBumperValues;                 //!< stores the foot bumper values; 0 for off, 1 for pressed
    
    // Buttons Sensors:
    sensor_t* ButtonValues;                     //!< stores the button values; 0 for unpressed, 1 for pressed
    
    // Battery Sensors:
    sensor_t* BatteryValues;                    //!< stores the battery values in Volts, Amperes and Watts
    
    // GPS Sensors
    sensor_t* GPS;                              //!< stores the gps position of the robot
    
private:
    vector<sensor_t*> m_sensors;                //!< a vector of all of the sensors
    vector<joint_id_t> m_head_ids;              //!< a vector of joint_id_t (index into sensor_t Joint*->Data) for each head joint
    vector<joint_id_t> m_larm_ids;              //!< a vector of joint_id_t (index into sensor_t Joint*->Data) for each left arm joint
    vector<joint_id_t> m_rarm_ids;              //!< a vector of joint_id_t (index into sensor_t Joint*->Data) for each right arm joint
    vector<joint_id_t> m_torso_ids;             //!< a vector of joint_id_t (index into sensor_t Joint*->Data) for each torso joint
    vector<joint_id_t> m_lleg_ids;              //!< a vector of joint_id_t (index into sensor_t Joint*->Data) for each left leg joint
    vector<joint_id_t> m_rleg_ids;              //!< a vector of joint_id_t (index into sensor_t Joint*->Data) for each right leg joint
};  

#endif

