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
    // kinematic sensors
    const static id_t LLegTransform;
    const static id_t RLegTransform;
    const static id_t SupportLegTransform;
    const static id_t CameraTransform;
    const static id_t CameraToGroundTransform;
    const static id_t CameraHeight;
    const static id_t Odometry;
    // balance sensors
    const static id_t Accelerometer;
    const static id_t Gyro;
    const static id_t GyroOffset;
    const static id_t Orientation;
    const static id_t Horizon;
    const static id_t Zmp;
    const static id_t Falling;
    const static id_t Fallen;
    // foot sensors
    const static id_t FootBumper;
    const static id_t FootForce;
    const static id_t FootContact;
    const static id_t FootSupport;
    const static id_t FootImpact;
    const static id_t FootCoP;
    // button sensors
    const static id_t MainButton;
    const static id_t SecondaryButton;
    const static id_t AllButton;
    const static id_t AllButtonTriggers;
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
public:
    NUSensorsData();
    ~NUSensorsData();
    
    void addSensors(const vector<string>& hardwarenames);
    
    // Get methods for a single joint at a time
    bool getJointPosition(id_t jointid, float& position);
    bool getJointVelocity(id_t jointid, float& velocity);
    bool getJointAcceleration(id_t jointid, float& acceleration);
    bool getJointTarget(id_t jointid, float& target);
    bool getJointStiffness(id_t jointid, float& stiffness);
    bool getJointCurrent(id_t jointid, float& current);
    bool getJointTorque(id_t jointid, float& torque);
    bool getJointTemperature(id_t jointid, float& temperature);
    
    // Get methods for a limb of joints (the limb can also be body and all)
    int getNumberOfJoints(id_t partid);
    bool getJointPositions(id_t bodypart, vector<float>& positions);
    bool getJointVelocities(id_t bodypart, vector<float>& velocities);
    bool getJointAccelerations(id_t bodypart, vector<float>& accelerations);
    bool getJointTargets(id_t bodypart, vector<float>& targets);
    bool getJointStiffnesses(id_t bodypart, vector<float>& stiffnesses);
    bool getJointCurrents(id_t bodypart, vector<float>& currents);
    bool getJointTorques(id_t bodypart, vector<float>& torques);
    bool getJointTemperatures(id_t bodypart, vector<float>& temperatures);
    bool getJointNames(id_t bodypart, vector<string>& names);
    
    // Get methods for soft proprioception
    bool getLeftLegTransform(Matrix& value);
    bool getRightLegTransform(Matrix& value);
    bool getSupportLegTransform(Matrix& value);
    bool getCameraTransform(Matrix& value);
    bool getCameraToGroundTransform(Matrix& value);

    bool getOdometry(float& time, vector<float>& values);
    bool getOdometryData(vector<float>& values);
    bool getCameraHeight(float& height);
    
    // Get methods for the other sensors
    bool getAccelerometerValues(vector<float>& values);
    bool getGyroValues(vector<float>& values);
    bool getGyroOffsetValues(vector<float>& values);
    bool getGyroFilteredValues(vector<float>& values);
    bool getOrientation(vector<float>& values);
    bool getOrientationHardware(vector<float>& values);
    bool getHorizon(vector<float>& values);
    bool getButtonTriggers(vector<float>& values);
    bool getZMP(vector<float>& values);
    bool getFalling(vector<float>& values);
    bool getFallen(vector<float>& values);
    bool getDistanceLeftValues(vector<float>& values);
    bool getDistanceRightValues(vector<float>& values);
    bool getBatteryValues(vector<float>& values);
    bool getGPSValues(vector<float>& values);
    bool getCompassValues(vector<float>& values);
    
    // Get methods for foot pressure sensors
    bool getFootSoleValues(id_t footid, vector<float>& values);
    bool getFootBumperValues(id_t footid, vector<float>& values);
    bool getFootCoP(id_t footid, float& x, float& y);
    bool getFootForce(id_t footid, float& force);
    bool getFootContact(id_t footid, bool& contact);
    bool getFootSupport(id_t footid, bool& support);
    bool getButtonValues(id_t buttonid, vector<float>& values);
    
    // Common sub-get methods
    bool isFalling();
    bool isFallen();
    bool isOnGround();
    bool isIncapacitated();
    bool footImpact(id_t footid, float& time);
    
    // Motion sensor get methods
    bool getMotionFallActive(bool& active);
    bool getMotionGetupActive(bool& active);
    bool getMotionKickActive(bool& active);
    bool getMotionSaveActive(bool& active);
    bool getMotionScriptActive(bool& active);
    bool getMotionWalkSpeed(vector<float>& speed);
    bool getMotionWalkMaxSpeed(vector<float>& speed);
    bool getMotionHeadCompletionTime(double& time);
    
    // Set methods for joints
    void set(const id_t& id, double time, const float& data);
    void set(const id_t& id, double time, const vector<float>& data);
    void set(const id_t& id, double time, const vector<vector<float> >& data);
    void set(const id_t& id, double time, const string& data);
    
    void setVelocity(const id_t& id, double time, const vector<float>& data);
    void setAcceleration(const id_t& id, double time, const vector<float>& data);
    void setTarget(const id_t& id, double time, const vector<float>& data);
    void setStiffness(const id_t& id, double time, const vector<float>& data);
    void setCurrent(const id_t& id, double time, const vector<float>& data);
    void setTemperature(const id_t& id, double time, const vector<float>& data);
    
    void setAsInvalid(const id_t& id);
    
    
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
    void setBalanceOrientationHardware(double time, const vector<float>& data, bool iscalculated = false);
    void setDistanceLeftValues(double time, const vector<float>& data, bool iscalculated = false);
    void setDistanceRightValues(double time, const vector<float>& data, bool iscalculated = false);
    void setFootSoleValues(double time, const vector<float>& data, bool iscalculated = false);
    void setFootBumperValues(double time, const vector<float>& data, bool iscalculated = false);
    void setButtonValues(double time, const vector<float>& data, bool iscalculated = false);
    void setBatteryValues(double time, const vector<float>& data, bool iscalculated = false);
    void setGPSValues(double time, const vector<float>& data, bool iscalculated = false);
    void setCompassValues(double time, const vector<float>& data, bool iscalculated = false);
    
    // Set methods for motion 'sensors'
    void setMotionFallActive(double time, bool active);
    void setMotionGetupActive(double time, bool active);
    void setMotionKickActive(double time, bool active);
    void setMotionSaveActive(double time, bool active);
    void setMotionScriptActive(double time, bool active);
    void setMotionWalkSpeed(double time, vector<float>& speed);
    void setMotionWalkMaxSpeed(double time, vector<float>& speed);
    void setMotionHeadCompletionTime(double time, double completiontime);
    
    void summaryTo(ostream& output) const;
    void csvTo(ostream& output);
    
    friend ostream& operator<< (ostream& output, const NUSensorsData& p_sensor);
    friend istream& operator>> (istream& input, NUSensorsData& p_sensor);
    
    int size() const;
    double GetTimestamp() const {return CurrentTime;};
public:
    double CurrentTime;                         //!< stores the most recent time sensors were updated in milliseconds
    
private:
    static vector<id_t*> m_ids;					//!< a vector containing all of the actionator ids
    vector<Sensor> m_sensors;                //!< a vector of all of the sensors
};  

#endif

