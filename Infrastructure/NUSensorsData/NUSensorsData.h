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

#include "Sensor.h"
#include "Infrastructure/NUData.h"

#include <vector>
#include <string>
#include "Tools/Math/Matrix.h"
#include "Tools/FileFormats/TimestampedData.h"
using namespace std;

class NUSensorsData: public NUData, TimestampedData
{
public:
    // end effector sensors
    const static id_t LArmEndEffector;                  // internal use only: Compactly stores the Bumper, Force, Contact, Support, Impact and Centre of Pressure of the end effector
    const static id_t RArmEndEffector;                  // internal use only                  
    const static id_t LLegEndEffector;                  // internal use only
    const static id_t RLegEndEffector;                  // internal use only
    // kinematic sensors
    const static id_t LLegTransform;                    // internal use only
    const static id_t RLegTransform;                    // internal use only
    const static id_t SupportLegTransform;              // internal use only
    const static id_t CameraTransform;                  // internal use only
    const static id_t CameraToGroundTransform;          // internal use only
    const static id_t CameraHeight;
    const static id_t Odometry;
    // balance sensors
    const static id_t Accelerometer;
    const static id_t Gyro;                             // raw gyro readings
    const static id_t GyroOffset;                       // gyro offset such that the filtered gyro values are gyro - gyro_offset
    const static id_t Orientation;
    const static id_t Horizon;
    const static id_t Zmp;
    const static id_t Falling;                          // Compactly stores Falling, Falling Left, Falling Right, Falling Forward, Falling Backward
    const static id_t Fallen;                           // Compactly stores Fallen, Fallen Left, Fallen Right, Fallen Forward, Fallen Backward
    // touch sensors
    const static id_t LHandTouch;                       // internal use only: Stores raw force readings from hardware array in left hand
    const static id_t RHandTouch;                       // internal use only: Stores raw force readings from hardware array in right hand
    const static id_t LFootTouch;                       // internal use only: Stores raw force readings from hardware array in left foot
    const static id_t RFootTouch;                       // internal use only: Stores raw force readings from hardware array in right foot
    // button sensors
    const static id_t MainButton;
    const static id_t LeftButton;
    const static id_t RightButton;
    const static id_t AllButton;                    // TODO: this is exceedingly difficult at the moment because I don't know how to make groups for sensors yet :(
    const static id_t AllButtonDurations;
    // distance sensors
    const static id_t LDistance;
    const static id_t RDistance;
    // gps sensors
    const static id_t Gps;
    const static id_t Compass;
    // battery sensors
    const static id_t BatteryVoltage;
    const static id_t BatteryCurrent;
    const static id_t BatteryCharge;
    // motion sensors
    const static id_t MotionFallActive;
    const static id_t MotionGetupActive;
    const static id_t MotionKickActive;
    const static id_t MotionSaveActive;
    const static id_t MotionScriptActive;
    const static id_t MotionWalkSpeed;
    const static id_t MotionWalkMaxSpeed;
    const static id_t MotionHeadCompletionTime;
    
    enum JointSensorIndices 
    {   // indices into the single joint vector
        PositionId = 0,
        VelocityId = 1,
        AccelerationId = 2,
        TargetId = 3,
        StiffnessId = 4,
        CurrentId = 5,
        TorqueId = 6,
        TemperatureId = 7,
        NumJointSensorIndices = 8
    };
    enum EndEffectorIndices
    {   // indices into a single end effector vector
        Bumper = 0,
        Force = 1,
        Contact = 2, 
        Support = 3, 
        Impact = 4, 
        CoP = 5,
        NumEndEffectorIndices = 6
    };
public:
    NUSensorsData();
    ~NUSensorsData();
    
    void addSensors(const vector<string>& hardwarenames);
    
    // Get methods for joint information
    bool getPosition(const id_t id, float& data);
    bool getPosition(const id_t id, vector<float>& data);
    bool getVelocity(const id_t id, float& data);
    bool getVelocity(const id_t id, vector<float>& data);
    bool getAcceleration(const id_t id, float& data);
    bool getAcceleration(const id_t id, vector<float>& data);
    bool getTarget(const id_t id, float& data);
    bool getTarget(const id_t id, vector<float>& data);
    bool getStiffness(const id_t id, float& data);
    bool getStiffness(const id_t id, vector<float>& data);
    bool getCurrent(const id_t id, float& data);
    bool getCurrent(const id_t id, vector<float>& data);
    bool getTorque(const id_t id, float& data);
    bool getTorque(const id_t id, vector<float>& data);
    bool getTemperature(const id_t id, float& data);
    bool getTemperature(const id_t id, vector<float>& data);
                     
    bool getEndPosition(const id_t id, vector<float>& data);
    bool getCameraHeight(float& data);
    bool getHorizon(vector<float>& data);
    bool getOdometry(vector<float>& data);
    
    bool getAccelerometer(vector<float>& data);
    bool getGyro(vector<float>& data);
    bool getOrientation(vector<float>& data);
    bool getFalling(vector<float>& data);
    bool getFallen(vector<float>& data);
    
    bool getBumper(const id_t& id, float& data);
    bool getForce(const id_t& id, float& data);
    bool getContact(const id_t& id, float& data);
    bool getSupport(const id_t& id, float& data);
    bool getImpact(const id_t& id, float& data);
    bool getZmp(const id_t& id, vector<float>& data);
    bool getCoP(const id_t& id, vector<float>& data);
    
    bool getButton(const id_t& id, float& data);
    bool getButton(const id_t& id, vector<float>& data);
    bool getButtonDuration(const id_t& id, float& data);
    bool getButtonDuration(const id_t& id, vector<float>& data);
    
    bool getDistance(const id_t& id, float& data);
    bool getDistance(const id_t& id, vector<float>& data);
    
    bool getGps(vector<float>& data);
    bool getCompass(vector<float>& data);
    
    bool getBatteryVoltage(float& data);
    bool getBatteryCurrent(float& data);
    bool getBatteryCharge(float& data);

    // Common sub-get methods
    bool isFalling();
    bool isFallen();
    bool isOnGround();
    bool isIncapacitated();
    bool footImpact(id_t footid, float& time);
    
    // Get Methods (generic) internal use only
    bool get(const id_t& id, bool& data);
    bool get(const id_t& id, float& data);
    bool get(const id_t& id, double& data);
    bool get(const id_t& id, vector<float>& data);
    bool get(const id_t& id, vector<vector<float> >& data);
    bool get(const id_t& id, string& data);
    
    // Set methods
    void set(const id_t& id, double time, bool data);
    void set(const id_t& id, double time, const float& data);
    void set(const id_t& id, double time, const vector<float>& data);
    void set(const id_t& id, double time, const vector<vector<float> >& data);
    void set(const id_t& id, double time, const string& data);
    void setAsInvalid(const id_t& id);
    
    void summaryTo(ostream& output) const;
    void csvTo(ostream& output);
    
    friend ostream& operator<< (ostream& output, const NUSensorsData& p_sensor);
    friend istream& operator>> (istream& input, NUSensorsData& p_sensor);
    
    int size() const;
    double GetTimestamp() const {return CurrentTime;};
private:
    bool getJointData(const id_t& id, const JointSensorIndices& in, float& data);
    bool getJointData(const id_t& id, const JointSensorIndices& in, vector<float>& data);
    bool getEndEffectorData(const id_t& id, const EndEffectorIndices& in, float& data);
public:
    double CurrentTime;                         //!< stores the most recent time sensors were updated in milliseconds

private:
    static vector<id_t*> m_ids;					//!< a vector containing all of the actionator ids
    vector<Sensor> m_sensors;                //!< a vector of all of the sensors
};  

#endif

