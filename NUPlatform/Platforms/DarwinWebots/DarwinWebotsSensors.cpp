/*! @file DarwinWebotsSensors.cpp
    @brief Implementation of Darwin in Webots sensor class

    @author Jason Kulk, Jed Rietveld
 
  Copyright (c) 2012 Jason Kulk, Jed Rietveld
 
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

#include "DarwinWebotsSensors.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Tools/Math/General.h"

#include "debug.h"
#include "debugverbositynusensors.h"

#include <limits>
using namespace webots;

// Apparently the best way to initialise a vector like an array, is to initialise the vector from an array

// init m_servo_names:
static std::string temp_servo_names[] = {std::string("HeadPitch"), std::string("HeadYaw"), \
                                    std::string("LShoulderRoll"), std::string("LShoulderPitch"), std::string("LElbowPitch"), \
                                    std::string("RShoulderRoll"), std::string("RShoulderPitch"), std::string("RElbowPitch"), \
                                    std::string("LHipRoll"),  std::string("LHipPitch"), std::string("LHipYaw"), \
									std::string("LKneePitch"), std::string("LAnkleRoll"), std::string("LAnklePitch"), \
                                    std::string("RHipRoll"),  std::string("RHipPitch"), std::string("RHipYaw"), \
									std::string("RKneePitch"), std::string("RAnkleRoll"), std::string("RAnklePitch")};
std::vector<std::string> DarwinWebotsSensors::m_servo_names(temp_servo_names, temp_servo_names + sizeof(temp_servo_names)/sizeof(*temp_servo_names));


/*! @brief Constructs a nubot sensor class with Webots backend
 
    @param platform a pointer to the nuplatform (this is required because webots needs to have nuplatform inherit from the Robot class)
 */
DarwinWebotsSensors::DarwinWebotsSensors(DarwinWebotsPlatform* platform) : m_simulation_step(int(platform->getBasicTimeStep()))
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "DarwinWebotsSensors::DarwinWebotsSensors()" << std::endl;
#endif
    m_platform = platform;
    getSensorsFromWebots();
    enableSensorsInWebots();
    m_data->addSensors(m_servo_names);
    m_joint_ids = m_data->mapIdToIds(NUSensorsData::All);
    m_previous_positions = std::vector<float>(m_servo_names.size(), 0);
    m_previous_velocities = std::vector<float>(m_servo_names.size(), 0);
}

/*! @brief Gets pointers to each of the sensors in the simulated Darwin
 */
void DarwinWebotsSensors::getSensorsFromWebots()
{
    // Get the servos
    for (size_t i=0; i<m_servo_names.size(); i++)
        m_servos.push_back(m_platform->getServo(m_servo_names[i]));
    // Get the accelerometer
    m_accelerometer = m_platform->getAccelerometer("accelerometer");
    // Get gyro
    m_gyro = m_platform->getGyro("gyro");
    // Get the gps if avaliable
    if (GPS::exists("gps"))
        m_gps = m_platform->getGPS("gps");
    else
        m_gps = NULL;
    
    // Get the compass if avaliable
    if (Compass::exists("compass"))
    {
        m_compass = m_platform->getCompass("compass");
    }
    else
        m_compass = NULL;
}

/* @brief Enables the sensor feedback in the simulated Darwin (in webots this must be done, otherwise the readings are never updated)
 */
void DarwinWebotsSensors::enableSensorsInWebots()
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

/*! @brief Destructor for DarwinWebotsSensors
 */
DarwinWebotsSensors::~DarwinWebotsSensors()
{
}

/*! @brief Gets the sensor data using the Webots API and puts it in the NUSensorsData data member.
 */
void DarwinWebotsSensors::copyFromHardwareCommunications()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "DarwinWebotsSensors::copyFromHardwareCommunications()" << std::endl;
#endif
    copyFromJoints();
    copyFromAccelerometerAndGyro();
    //copyFromDistance();
    //copyFromFootSole();
    //copyFromFootBumper();
    //copyFromGPS();
    //copyFromCompass();
}

/*! @brief Copies the joint data into m_data
 */
void DarwinWebotsSensors::copyFromJoints()
{   
	#if DEBUG_NUSENSORS_VERBOSITY > 4
        debug << "DarwinWebotsSensors::copyFromJoints()" << std::endl;
    #endif
    static float NaN = numeric_limits<float>::quiet_NaN();

    std::vector<float> joint(NUSensorsData::NumJointSensorIndices, NaN);
    float delta_t = (m_current_time - m_previous_time)/1000;  
	for (size_t i=0; i<m_servos.size(); i++)
    {
        JServo* s = static_cast<JServo*>(m_servos[i]);
        joint[NUSensorsData::PositionId] = DarwinJointMapping::Instance().rad2joint(i, s->getPosition());           
        joint[NUSensorsData::VelocityId] = (joint[NUSensorsData::PositionId] - m_previous_positions[i])/delta_t;    
        joint[NUSensorsData::AccelerationId] = (joint[NUSensorsData::VelocityId] - m_previous_velocities[i])/delta_t;
        joint[NUSensorsData::TargetId] = DarwinJointMapping::Instance().rad2joint(i, s->getTargetPosition());       
        joint[NUSensorsData::StiffnessId] = s->getTargetGain();        
        joint[NUSensorsData::TorqueId] = s->getMotorForceFeedback();   
        m_data->set(*m_joint_ids[i], m_current_time, joint);
        
        m_previous_positions[i] = joint[NUSensorsData::PositionId];
        m_previous_velocities[i] = joint[NUSensorsData::VelocityId];
    }
}

/*! @brief Copies the accelerometer and gyro data into m_data
 */
void DarwinWebotsSensors::copyFromAccelerometerAndGyro()
{
    const unsigned char numdimensions = 3;
    static std::vector<float> accelerometerdata(numdimensions, 0);
    static std::vector<float> gyrodata(numdimensions, 0);
	
	float VALUETORPS_RATIO = 18.3348;//512/27.925
    float VALUETOACCEL_RATIO = 0.1304; //512/4*981
    float centrevalue = 512;
    
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "DarwinWebotsSensors::copyFromAccelerometerAndGyro()" << std::endl;
#endif
    
    // Copy accelerometer [ax, ay, az]
    static const double *buffer;
    buffer = m_accelerometer->getValues();
    //for (size_t i=0; i<numdimensions; i++)
    accelerometerdata[0] = (centrevalue-(buffer[1]))/VALUETOACCEL_RATIO;
	accelerometerdata[1] = (centrevalue-(buffer[0]))/VALUETOACCEL_RATIO; 
	accelerometerdata[2] = (centrevalue-(buffer[2]))/VALUETOACCEL_RATIO;  


	// convert from m/s/s to cm/s/s, and swap sign as it is incorrect in webots
    m_data->set(NUSensorsData::Accelerometer, m_current_time, accelerometerdata);
    // Copy gyro [gx, gy, gz]
    buffer = m_gyro->getValues();
    for (size_t i=0; i<numdimensions; i++)
        gyrodata[i] = (centrevalue-buffer[i])/VALUETORPS_RATIO;
    m_data->set(NUSensorsData::Gyro, m_current_time, gyrodata);
}

/*! @brief Copies the distance data into m_data
 */
void DarwinWebotsSensors::copyFromDistance()
{
    static std::vector<float> leftdistance(1,0);
    static std::vector<float> rightdistance(1,0);
    
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "DarwinWebotsSensors::copyFromDistance()" << std::endl;
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
    
    m_data->set(NUSensorsData::LDistance, m_current_time, leftdistance);
    m_data->set(NUSensorsData::RDistance, m_current_time, rightdistance);
}

/*! @brief Copies the foot sole pressure data into m_data
 */
void DarwinWebotsSensors::copyFromFootSole()
{
    static std::vector<float> lfootsoledata(m_foot_sole_sensors.size()/2, 0);
    static std::vector<float> rfootsoledata(m_foot_sole_sensors.size()/2, 0);
    
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "DarwinWebotsSensors::copyFromFootSole()" << std::endl;
#endif
    
    // Copy foot sole readings
    size_t midpoint = m_foot_sole_sensors.size()/2;
    for (size_t i=0; i<midpoint; i++)
        lfootsoledata[i] = m_foot_sole_sensors[i]->getValue();
    for (size_t i=midpoint; i<m_foot_sole_sensors.size(); i++)
        rfootsoledata[i-midpoint] = m_foot_sole_sensors[i]->getValue();
    m_data->set(NUSensorsData::LFootTouch, m_current_time, lfootsoledata);
    m_data->set(NUSensorsData::RFootTouch, m_current_time, rfootsoledata);
}

/*! @brief Copies the foot bumper data into m_data
 */
void DarwinWebotsSensors::copyFromFootBumper()
{
    #if DEBUG_NUSENSORS_VERBOSITY > 4
        debug << "DarwinWebotsSensors::copyFromFootBumper()" << std::endl;
    #endif
    float leftbumper = m_foot_bumper_sensors[0]->getValue() + m_foot_bumper_sensors[1]->getValue();
    float rightbumper =  m_foot_bumper_sensors[2]->getValue() + m_foot_bumper_sensors[3]->getValue();
    m_data->modify(NUSensorsData::LLegEndEffector, NUSensorsData::BumperId, m_current_time, leftbumper);
    m_data->modify(NUSensorsData::RLegEndEffector, NUSensorsData::BumperId, m_current_time, rightbumper);
}

/*! @brief Copies the gps data into m_data
 */
void DarwinWebotsSensors::copyFromGPS()
{
    const unsigned char numdimensions = 3;
    if (m_gps != NULL)
    {
        static std::vector<float> gpsdata(numdimensions, 0);
        static const double *buffer;
        
        #if DEBUG_NUSENSORS_VERBOSITY > 4
            debug << "DarwinWebotsSensors::copyFromGPS()" << std::endl;
        #endif
        
        buffer = m_gps->getValues();
        gpsdata[0] = 100*buffer[0];                // the data from webots is: [x, z, -y]
        gpsdata[1] = -100*buffer[2];
        gpsdata[2] = 100*buffer[1];
        m_data->set(NUSensorsData::Gps, m_current_time, gpsdata);
    }
}

/*! @brief Copies the compass data into m_data
 */
void DarwinWebotsSensors::copyFromCompass()
{
    if (m_compass != NULL)
    {
        static const double *buffer;
        
        #if DEBUG_NUSENSORS_VERBOSITY > 4
            debug << "DarwinWebotsSensors::copyFromCompass()" << std::endl;
        #endif
        
        buffer = m_compass->getValues();
        float compassdata = mathGeneral::normaliseAngle(atan2(buffer[0], buffer[1]) - 1.5708);
        m_data->set(NUSensorsData::Compass, m_current_time, compassdata);
    }
}



