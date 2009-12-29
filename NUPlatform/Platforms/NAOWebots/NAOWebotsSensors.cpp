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
#include "Tools/debug.h"

// Apparently the best way to initialise a vector like an array, is to initialise the vector from an array

// init m_servo_names:
static string temp_servo_names[] = {string("HeadYaw"), string("HeadPitch"), \
                                    string("LShoulderPitch"), string("LShoulderRoll"), string("LElbowYaw"), string("LElbowRoll"), \
                                    string("RShoulderPitch"), string("RShoulderRoll"), string("RElbowYaw"), string("RElbowRoll"), \
                                    string("LHipYawPitch"), string("LHipPitch"), string("LHipRoll"), string("LKneePitch"), string("LAnklePitch"), string("LAnkleRoll"), \
                                    string("RHipYawPitch"), string("RHipPitch"), string("RHipRoll"), string("RKneePitch"), string("RAnklePitch"), string("RAnkleRoll")};
vector<string> NAOWebotsSensors::m_servo_names(temp_servo_names, temp_servo_names + sizeof(temp_servo_names)/sizeof(*temp_servo_names));

// init m_distance_names:
static string temp_distance_names[] = {string("US/TopLeft"), string("US/BottomLeft"), string("US/TopRight"), string("US/BottomRight")};
vector<string> NAOWebotsSensors::m_distance_names(temp_distance_names, temp_distance_names + sizeof(temp_distance_names)/sizeof(*temp_distance_names));

// init m_foot_names:
static string temp_foot_sole_names[] = {string("LFsrFL"), string("LFsrFR"), string("LFsrBR"), string("LFsrBL"), \
                                    string("RFsrFL"), string("RFsrFR"), string("RFsrBR"), string("RFsrBL")};
vector<string> NAOWebotsSensors::m_foot_sole_names(temp_foot_sole_names, temp_foot_sole_names + sizeof(temp_foot_sole_names)/sizeof(*temp_foot_sole_names));

// init m_button_names:
static string temp_foot_bumper_names[] = {string("LFoot/Bumper/Left"), string("LFoot/Bumper/Right"), string("RFoot/Bumper/Left"), string("RFoot/Bumper/Right")};
vector<string> NAOWebotsSensors::m_foot_bumper_names(temp_foot_bumper_names, temp_foot_bumper_names + sizeof(temp_foot_bumper_names)/sizeof(*temp_foot_bumper_names));

/*! @brief Constructs a nubot sensor class with Webots backend
 
    @param platform a pointer to the nuplatform (this is required because webots needs to have nuplatform inherit from the Robot class)
 */
NAOWebotsSensors::NAOWebotsSensors(NAOWebotsPlatform* platform) : m_simulation_step(platform->getBasicTimeStep())
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
    for (int i=0; i<m_servo_names.size(); i++)
        m_servos.push_back(m_platform->getServo(m_servo_names[i]));
    // Get the accelerometer
    m_accelerometer = m_platform->getAccelerometer("accelerometer");
    // Get gyro
    m_gyro = m_platform->getGyro("gyro");
    // Get distance sensors
    for (int i=0; i<m_distance_names.size(); i++)
        m_distance_sensors.push_back(m_platform->getDistanceSensor(m_distance_names[i]));
    // Get foot sole sensors
    for (int i=0; i<m_foot_sole_names.size(); i++)
        m_foot_sole_sensors.push_back(m_platform->getTouchSensor(m_foot_sole_names[i]));
    // Get foor bumper sensors
    for (int i=0; i<m_foot_bumper_names.size(); i++)
        m_foot_bumper_sensors.push_back(m_platform->getTouchSensor(m_foot_bumper_names[i]));
    
    // Get the gps if avaliable
    if (GPS::exists("gps"))
        m_gps = m_platform->getGPS("gps");
    else
        m_gps = NULL;
}

/* @brief Enables the sensor feedback in the simulated NAO (in webots this must be done, otherwise the readings are never updated)
 */
void NAOWebotsSensors::enableSensorsInWebots()
{
    // Enable the position and torque feedback
    for (int i=0; i<m_servos.size(); i++)
    {
        m_servos[i]->enablePosition(m_simulation_step);
        m_servos[i]->enableMotorForceFeedback(m_simulation_step);
    }
    // Enable the accelerometer
    m_accelerometer->enable(m_simulation_step);
    // Enable the  gyro
    m_gyro->enable(m_simulation_step);
    // Enable the distance sensors
    for (int i=0; i<m_distance_sensors.size(); i++)
        m_distance_sensors[i]->enable(m_simulation_step);
    // Enable the foot sole sensors
    for (int i=0; i<m_foot_sole_sensors.size(); i++)
        m_foot_sole_sensors[i]->enable(m_simulation_step);
    // Enable the foor bumper sensors
    for (int i=0; i<m_foot_bumper_sensors.size(); i++)
        m_foot_bumper_sensors[i]->enable(m_simulation_step);
    
    // Enable the the gps if avaliable
    if (m_gps != NULL)
        m_gps->enable(m_simulation_step);
}

/*! @brief Destructor for NAOWebotsSensors
 */
NAOWebotsSensors::~NAOWebotsSensors()
{
    m_servo_names.clear();
    m_servos.clear();
    delete m_accelerometer;
    delete m_gyro;
    m_distance_names.clear();
    m_distance_sensors.clear();
    m_foot_sole_names.clear();
    m_foot_sole_sensors.clear();
    m_foot_bumper_names.clear();
    m_foot_bumper_sensors.clear();
    if (m_gps != NULL)
        delete m_gps;
}

/*! @brief Gets the sensor data using the Webots API and puts it in the NUSensorsData data member.
 */
void NAOWebotsSensors::copyFromHardwareCommunications()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NAOWebotsSensors::copyFromHardwareCommunications()" << endl;
#endif

    m_current_time = nusystem->getTime();             // hmmmm. I am not sure whether I am allowed to do this. Its a backdoor so I don't like it!
    
    copyFromJoints();
    copyFromAccelerometerAndGyro();
    copyFromDistance();
    copyFromFootSole();
    copyFromFootBumper();
    copyFromGPS();
    
#if DEBUG_NUSENSORS_VERBOSITY > 3
    static bool firstrun = true;
    if (firstrun)
    {
        debug << "NAOWebotsSensors::NAOWebotsSensors(). Available Sensors:" << endl;
        m_data->summaryTo(debug);
        firstrun = false;
    }
#endif
#if DEBUG_NUSENSORS_VERBOSITY > 5
    debug << "NAOWebotsSensors::NAOWebotsSensors():" << endl;
    m_data->summaryTo(debug);
#endif
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
    for (int i=0; i<m_servos.size(); i++)
        positiondata[i] = m_servos[i]->getPosition();
    m_data->setJointPositions(m_current_time, positiondata);
    
    calculateJointVelocityFromPosition();
    calculateJointAccelerationFromVelocity();
    
    // Copy joint targets
    for (int i=0; i<m_servos.size(); i++)
        targetdata[i] = ((JServo*) m_servos[i])->getTargetPosition();
    m_data->setJointTargets(m_current_time, targetdata);
    
    // Copy joint stiffnesses
    for (int i=0; i<m_servos.size(); i++)
        stiffnessdata[i] = 0.1*((JServo*) m_servos[i])->getTargetGain();
    m_data->setJointStiffnesses(m_current_time, stiffnessdata);
    
    // Copy joint torques
    for (int i=0; i<m_servos.size(); i++)
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
    for (int i=0; i<numdimensions; i++)
        accelerometerdata[i] = 100*buffer[i];       // convert from m/s/s to cm/s/s
    m_data->setBalanceAccelerometer(m_current_time, accelerometerdata);
    // Copy gyro [gx, gy, gz]
    buffer = m_gyro->getValues();
    for (int i=0; i<numdimensions; i++)
        gyrodata[i] = buffer[i];
    m_data->setBalanceGyro(m_current_time, gyrodata);
}

/*! @brief Copies the distance data into m_data
 */
void NAOWebotsSensors::copyFromDistance()
{
    static vector<float> distancedata(m_distance_sensors.size(), 0);
    
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NAOWebotsSensors::copyFromDistance()" << endl;
#endif
    
    // Copy distance readings
    for (int i=0; i<m_distance_sensors.size(); i++)
        distancedata[i] = 100*m_distance_sensors[i]->getValue();        // convert from metres to centimetres
    m_data->setDistanceValues(m_current_time, distancedata);
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
    for (int i=0; i<m_foot_sole_sensors.size(); i++)
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
    for (int i=0; i<m_foot_bumper_sensors.size(); i++)
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
        for (int i=0; i<numdimensions; i++)
        {
            if (i == 0 || i == 1)
                gpsdata[i] = 100*buffer[i];         // convert to cm for gps coordinates
            else
                gpsdata[i] = buffer[i];
        }
        m_data->setGPSValues(m_current_time, gpsdata);      //! @todo TODO: add gps data to NUsensorsData
    }
}




