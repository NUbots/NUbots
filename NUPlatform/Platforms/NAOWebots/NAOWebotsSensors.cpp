/*! @file NAOWebotsSensors.cpp
    @brief Implementation of NAO in Webots sensor class

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

#include "NAOWebotsSensors.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"

#include "debug.h"
#include "debugverbositynusensors.h"
using namespace webots;

// Apparently the best way to initialise a vector like an array, is to initialise the vector from an array

// init m_servo_names:
static string temp_servo_names[] = {string("HeadPitch"), string("HeadYaw"), \
                                    string("LShoulderRoll"), string("LShoulderPitch"), string("LElbowRoll"), string("LElbowYaw"), \
                                    string("RShoulderRoll"), string("RShoulderPitch"), string("RElbowRoll"), string("RElbowYaw"), \
                                    string("LHipRoll"),  string("LHipPitch"), string("LHipYawPitch"), string("LKneePitch"), string("LAnkleRoll"), string("LAnklePitch"), \
                                    string("RHipRoll"),  string("RHipPitch"), string("RHipYawPitch"), string("RKneePitch"), string("RAnkleRoll"), string("RAnklePitch")};
vector<string> NAOWebotsSensors::m_servo_names(temp_servo_names, temp_servo_names + sizeof(temp_servo_names)/sizeof(*temp_servo_names));

// init m_distance_names:
static string temp_distance_names[] = {string("US/TopLeft"), string("US/BottomLeft"), string("US/TopRight"), string("US/BottomRight")};
vector<string> NAOWebotsSensors::m_distance_names(temp_distance_names, temp_distance_names + sizeof(temp_distance_names)/sizeof(*temp_distance_names));

// init m_foot_names:
static string temp_foot_sole_names[] = {string("LFsrFL"), string("LFsrFR"), string("LFsrBL"), string("LFsrBR"), \
                                    string("RFsrFL"), string("RFsrFR"), string("RFsrBL"), string("RFsrBR")};
vector<string> NAOWebotsSensors::m_foot_sole_names(temp_foot_sole_names, temp_foot_sole_names + sizeof(temp_foot_sole_names)/sizeof(*temp_foot_sole_names));

// init m_button_names:
static string temp_foot_bumper_names[] = {string("LFoot/Bumper/Left"), string("LFoot/Bumper/Right"), string("RFoot/Bumper/Left"), string("RFoot/Bumper/Right")};
vector<string> NAOWebotsSensors::m_foot_bumper_names(temp_foot_bumper_names, temp_foot_bumper_names + sizeof(temp_foot_bumper_names)/sizeof(*temp_foot_bumper_names));

/*! @brief Constructs a nubot sensor class with Webots backend
 
    @param platform a pointer to the nuplatform (this is required because webots needs to have nuplatform inherit from the Robot class)
 */
NAOWebotsSensors::NAOWebotsSensors(NAOWebotsPlatform* platform) : m_simulation_step(int(platform->getBasicTimeStep()))
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NAOWebotsSensors::NAOWebotsSensors()" << endl;
#endif
    m_platform = platform;
    getSensorsFromWebots();
    enableSensorsInWebots();
    m_data->setAvailableJoints(m_servo_names);
}

/*! @brief Gets pointers to each of the sensors in the simulated NAO
 */
void NAOWebotsSensors::getSensorsFromWebots()
{
    // Get the servos
    for (size_t i=0; i<m_servo_names.size(); i++)
        m_servos.push_back(m_platform->getServo(m_servo_names[i]));
    // Get the accelerometer
    m_accelerometer = m_platform->getAccelerometer("accelerometer");
    // Get gyro
    m_gyro = m_platform->getGyro("gyro");
    // Get distance sensors
    for (size_t i=0; i<m_distance_names.size(); i++)
        m_distance_sensors.push_back(m_platform->getDistanceSensor(m_distance_names[i]));
    // Get foot sole sensors
    for (size_t i=0; i<m_foot_sole_names.size(); i++)
        m_foot_sole_sensors.push_back(m_platform->getTouchSensor(m_foot_sole_names[i]));
    // Get foor bumper sensors
    for (size_t i=0; i<m_foot_bumper_names.size(); i++)
        m_foot_bumper_sensors.push_back(m_platform->getTouchSensor(m_foot_bumper_names[i]));
    
    // Get the gps if avaliable
    if (GPS::exists("gps"))
        m_gps = m_platform->getGPS("gps");
    else
        m_gps = NULL;
    
    // Get the compass if avaliable
    if (Compass::exists("compass"))
        m_compass = m_platform->getCompass("compass");
    else
        m_compass = NULL;
}

/* @brief Enables the sensor feedback in the simulated NAO (in webots this must be done, otherwise the readings are never updated)
 */
void NAOWebotsSensors::enableSensorsInWebots()
{
    // Enable the position and torque feedback
    for (size_t i=0; i<m_servos.size(); i++)
    {
        m_servos[i]->enablePosition(m_simulation_step);
        m_servos[i]->enableMotorForceFeedback(m_simulation_step);
    }
    // Enable the accelerometer
    m_accelerometer->enable(m_simulation_step);
    // Enable the  gyro
    m_gyro->enable(m_simulation_step);
    // Enable the distance sensors
    for (size_t i=0; i<m_distance_sensors.size(); i++)
        m_distance_sensors[i]->enable(m_simulation_step);
    // Enable the foot sole sensors
    for (size_t i=0; i<m_foot_sole_sensors.size(); i++)
        m_foot_sole_sensors[i]->enable(m_simulation_step);
    // Enable the foor bumper sensors
    for (size_t i=0; i<m_foot_bumper_sensors.size(); i++)
        m_foot_bumper_sensors[i]->enable(m_simulation_step);
    
    // Enable the the gps if avaliable
    if (m_gps != NULL)
        m_gps->enable(m_simulation_step);
    
    // Enable the compass if avaliable
    if (m_compass != NULL)
        m_compass->enable(m_simulation_step);
}

/*! @brief Destructor for NAOWebotsSensors
 */
NAOWebotsSensors::~NAOWebotsSensors()
{
}

/*! @brief Gets the sensor data using the Webots API and puts it in the NUSensorsData data member.
 */
void NAOWebotsSensors::copyFromHardwareCommunications()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NAOWebotsSensors::copyFromHardwareCommunications()" << endl;
#endif
    copyFromJoints();
    copyFromAccelerometerAndGyro();
    copyFromDistance();
    copyFromFootSole();
    copyFromFootBumper();
    copyFromGPS();
    copyFromCompass();
}

/*! @brief Copies the joint data into m_data
 */
void NAOWebotsSensors::copyFromJoints()
{
    static vector<float> positiondata(m_servos.size(), 0);
    static vector<float> velocitydata(m_servos.size(), 0);
    static vector<float> accelerationdata(m_servos.size(), 0);
    static vector<float> targetdata(m_servos.size(), 0);
    static vector<float> stiffnessdata(m_servos.size(), 0);
    static vector<float> torquedata(m_servos.size(), 0);
    
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NAOWebotsSensors::copyFromJoints()" << endl;
#endif

    // Copy joint positions
    for (size_t i=0; i<m_servos.size(); i++)
        positiondata[i] = m_servos[i]->getPosition();
    m_data->setJointPositions(m_current_time, positiondata);
    
    // Copy joint targets
    for (size_t i=0; i<m_servos.size(); i++)
        targetdata[i] = ((JServo*) m_servos[i])->getTargetPosition();
    m_data->setJointTargets(m_current_time, targetdata);
    
    // Copy joint stiffnesses
    for (size_t i=0; i<m_servos.size(); i++)
        stiffnessdata[i] = ((JServo*) m_servos[i])->getTargetGain();
    m_data->setJointStiffnesses(m_current_time, stiffnessdata);
    
    // Copy joint torques
    for (size_t i=0; i<m_servos.size(); i++)
        torquedata[i] = m_servos[i]->getMotorForceFeedback();
    m_data->setJointTorques(m_current_time, torquedata);
}

/*! @brief Copies the accelerometer and gyro data into m_data
 */
void NAOWebotsSensors::copyFromAccelerometerAndGyro()
{
    const unsigned char numdimensions = 3;
    static vector<float> accelerometerdata(numdimensions, 0);
    static vector<float> gyrodata(numdimensions, 0);
    
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NAOWebotsSensors::copyFromAccelerometerAndGyro()" << endl;
#endif
    
    // Copy accelerometer [ax, ay, az]
    static const double *buffer;
    buffer = m_accelerometer->getValues();
    for (size_t i=0; i<numdimensions; i++)
        accelerometerdata[i] = -100*buffer[i];       // convert from m/s/s to cm/s/s, and swap sign as it is incorrect in webots
    m_data->setBalanceAccelerometer(m_current_time, accelerometerdata);
    // Copy gyro [gx, gy, gz]
    buffer = m_gyro->getValues();
    for (size_t i=0; i<numdimensions; i++)
        gyrodata[i] = buffer[i];
    m_data->setBalanceGyro(m_current_time, gyrodata);
}

/*! @brief Copies the distance data into m_data
 */
void NAOWebotsSensors::copyFromDistance()
{
    static vector<float> leftdistance(1,0);
    static vector<float> rightdistance(1,0);
    
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NAOWebotsSensors::copyFromDistance()" << endl;
#endif
    
    // Copy left distance readings
    leftdistance[0] = 255;
    for (size_t i=0; i<m_distance_sensors.size()/2; i++)
    {
        float d = 100*m_distance_sensors[i]->getValue();
        if (d < 70 and d < leftdistance[0])
            leftdistance[0] = d;
    }
    // Copy right distance readings
    rightdistance[0] = 255;
    for (size_t i=m_distance_sensors.size()/2; i<m_distance_sensors.size(); i++)
    {
        float d = 100*m_distance_sensors[i]->getValue();
        if (d < 70 and d < rightdistance[0])
            rightdistance[0] = d;
    }
    
    m_data->setDistanceLeftValues(m_current_time, leftdistance);
    m_data->setDistanceRightValues(m_current_time, rightdistance);
}

/*! @brief Copies the foot sole pressure data into m_data
 */
void NAOWebotsSensors::copyFromFootSole()
{
    static vector<float> footsoledata(m_foot_sole_sensors.size(), 0);
    
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NAOWebotsSensors::copyFromFootSole()" << endl;
#endif
    
    // Copy foot sole readings
    for (size_t i=0; i<m_foot_sole_sensors.size(); i++)
        footsoledata[i] = m_foot_sole_sensors[i]->getValue();
    m_data->setFootSoleValues(m_current_time, footsoledata);
}

/*! @brief Copies the foot bumper data into m_data
 */
void NAOWebotsSensors::copyFromFootBumper()
{
    static vector<float> footbumperdata(m_foot_bumper_sensors.size(), 0);
    
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NAOWebotsSensors::copyFromFootBumper()" << endl;
#endif
    
    // Copy foot bumper readings
    for (size_t i=0; i<m_foot_bumper_sensors.size(); i++)
        footbumperdata[i] = m_foot_bumper_sensors[i]->getValue();
    m_data->setFootBumperValues(m_current_time, footbumperdata);
}

/*! @brief Copies the gps data into m_data
 */
void NAOWebotsSensors::copyFromGPS()
{
    const unsigned char numdimensions = 3;
    if (m_gps != NULL)
    {
        static vector<float> gpsdata(numdimensions, 0);
        static const double *buffer;
        
        #if DEBUG_NUSENSORS_VERBOSITY > 4
            debug << "NAOWebotsSensors::copyFromGPS()" << endl;
        #endif
        
        buffer = m_gps->getValues();
        gpsdata[0] = 100*buffer[0];                // the data from webots is: [x, z, -y]
        gpsdata[1] = -100*buffer[2];
        gpsdata[2] = 100*buffer[1];
        m_data->setGPSValues(m_current_time, gpsdata);
    }
}

/*! @brief Copies the compass data into m_data
 */
void NAOWebotsSensors::copyFromCompass()
{
    if (m_compass != NULL)
    {
        static vector<float> compassdata(1, 0);
        static const double *buffer;
        
        #if DEBUG_NUSENSORS_VERBOSITY > 4
            debug << "NAOWebotsSensors::copyFromCompass()" << endl;
        #endif
        
        buffer = m_compass->getValues();
        compassdata[0] = atan2(buffer[0], buffer[1]);
        m_data->setCompassValues(m_current_time, compassdata);
    }
}



