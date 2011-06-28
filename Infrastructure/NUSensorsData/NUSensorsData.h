/*! @file NUSensorsData.h
    @brief Declaration of a sensor data storage class to store sensor data in a platform independent way
    @author Jason Kulk
 
    @class NUSensorsData
    @brief A sensor class to store sensor data in a platform independent way
 
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

#ifndef NUSENSORSDATA_H
#define NUSENSORSDATA_H

#include "Sensor.h"
#include "Infrastructure/NUData.h"
#include "Tools/FileFormats/TimestampedData.h"
#include "NULocalisationSensors.h"

#include <vector>
#include <string>
using namespace std;

class NUSensorsData: public NUData, public TimestampedData
{
public:
    // end effector sensors
    const static id_t LArmEndEffector;                  // internal use only: Compactly stores the Bumper, Force, Contact, Support, Impact and Centre of Pressure of the end effector
    const static id_t RArmEndEffector;                  // internal use only: If an end effector does not have a particular sub-sensor, then it will be NaN (like the joint informations).                  
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
    const static id_t Orientation;                      // orientation sensor
    const static id_t OrientationHardware;              // a hardware orientation sensor
    const static id_t Horizon;                          // a sensor for the horizon line
    const static id_t Zmp;                              // a zero moment point sensor
    const static id_t Falling;                          // Compactly stores Falling, Falling Left, Falling Right, Falling Forward, Falling Backward
    const static id_t Fallen;                           // Compactly stores Fallen, Fallen Left, Fallen Right, Fallen Forward, Fallen Backward
    // touch sensors
    const static id_t LHandTouch;                       // internal use only: Stores raw force readings from hardware array in left hand
    const static id_t RHandTouch;                       // internal use only: Stores raw force readings from hardware array in right hand
    const static id_t LFootTouch;                       // internal use only: Stores raw force readings from hardware array in left foot
    const static id_t RFootTouch;                       // internal use only: Stores raw force readings from hardware array in right foot
    // button sensors
    const static id_t MainButton;						// Compactly stores the [state, duration] for the main button
    const static id_t LeftButton;						// Compactly stores the [state, duration] for the left button
    const static id_t RightButton;						// Compactly stores the [state, duration] for the right button
    // distance sensors
    const static id_t LDistance;						// Leftward facing ultrasonic sensor
    const static id_t RDistance;						// Rightward facing ultrasonic sensor
    const static id_t LaserDistance;					// Laser scanner or similar sensor (infrared array, 2d laser, 3d laser/infrared)
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
    
    const static unsigned int m_num_sensor_ids;                  //!< internal use only.
    
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
        BumperId = 0,
        ForceId = 1,
        ContactId = 2, 
        SupportId = 3, 
        CoPXId = 4,
        CoPYId = 5,
        EndPositionXId = 6,
        EndPositionYId = 7, 
        EndPositionZId = 8,
        EndPositionRollId = 9,
        EndPositionPitchId = 10,
        EndPositionYawId = 11,
        NumEndEffectorIndices = 12
    };
    enum ButtonSensorIndices
    {	// indices into a single button vector
        StateId = 0,	
        DurationId = 1,
        NumButtonIndices = 3
    };
public:
    NUSensorsData();
    NUSensorsData(const NULocalisationSensors& locsensors);
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
    
    // Get methods for end effector information
    bool getBumper(const id_t& id, float& data);
    bool getForce(const id_t& id, float& data);
    bool getContact(const id_t& id, bool& data);
    bool getSupport(const id_t& id, bool& data);
    bool getCoP(const id_t& id, vector<float>& data);
    bool getEndPosition(const id_t id, vector<float>& data);
    
    // Get methods for kinematic based information
    bool getCameraHeight(float& data);
    bool getHorizon(vector<float>& data);
    bool getOdometry(vector<float>& data);
    
    // Get methods for balance information
    bool getAccelerometer(vector<float>& data);
    bool getGyro(vector<float>& data);
    bool getOrientation(vector<float>& data);
    bool getFalling(vector<float>& data);
    bool getFallen(vector<float>& data);
    bool getZmp(const id_t& id, vector<float>& data);
    
    // Get methods for other sensors
    bool getGps(vector<float>& data);
    bool getCompass(float& data);
    bool getDistance(const id_t& id, vector<float>& data);
    
    bool getButton(const id_t& id, float& data);
    bool getButtonDuration(const id_t& id, float& data);
    
    bool getBatteryVoltage(float& data);
    bool getBatteryCurrent(float& data);
    bool getBatteryCharge(float& data);

    // Common sub-get methods
    bool isFalling();
    bool isFallen();
    bool isOnGround();
    bool isIncapacitated();
    
    // Get Methods (generic) internal use only
    bool get(const id_t& id, bool& data);
    bool get(const id_t& id, float& data);
    bool get(const id_t& id, double& data);
    bool get(const id_t& id, vector<float>& data);
    bool get(const id_t& id, vector<vector<float> >& data);
    bool get(const id_t& id, string& data);
    
    
    
    // Set methods (generic) internal use only
    void set(const id_t& id, double time, bool data);
    void set(const id_t& id, double time, const float& data);
    void set(const id_t& id, double time, const vector<float>& data);
    void set(const id_t& id, double time, const vector<vector<float> >& data);
    void set(const id_t& id, double time, const string& data);
    void setAsInvalid(const id_t& id);    
    void modify(const id_t& id, int start, double time, const float& data);
    void modify(const id_t& id, int start, double time, const vector<float>& data);
    
    void summaryTo(ostream& output) const;
    void csvTo(ostream& output);
    
    friend ostream& operator<< (ostream& output, const NUSensorsData& p_sensor);
    friend istream& operator>> (istream& input, NUSensorsData& p_sensor);
    
    int size() const;
    double GetTimestamp() const {return CurrentTime;}
    NULocalisationSensors getLocSensors();
private:
    bool getJointData(const id_t& id, const JointSensorIndices& in, float& data);
    bool getJointData(const id_t& id, const JointSensorIndices& in, vector<float>& data);
    bool getEndEffectorData(const id_t& id, const EndEffectorIndices& in, float& data);
    bool getButtonData(const id_t& id, const ButtonSensorIndices& in, float& data);

private:
    static vector<id_t*> m_ids;				 //!< a vector containing all of the actionator ids
    vector<Sensor> m_sensors;                //!< a vector of all of the sensors
};  

#endif

