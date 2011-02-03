/*! @file NAOSensors.cpp
    @brief Implementation of NAO sensor class

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

#include "NAOSensors.h"
#include "NAOSensorNames.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "NUNAO.h"

#include "Tools/Profiling/Profiler.h"

#include "debug.h"
#include "debugverbositynusensors.h"

#include <limits>
using namespace std;

// Apparently the best way to initialise a vector like an array, is to initialise the vector from an array
static string temp_jointposition_names[] = {HEAD_PITCH_POSITION, HEAD_YAW_POSITION, L_SHOULDER_ROLL_POSITION, L_SHOULDER_PITCH_POSITION, L_ELBOW_ROLL_POSITION, L_ELBOW_YAW_POSITION, R_SHOULDER_ROLL_POSITION, R_SHOULDER_PITCH_POSITION, R_ELBOW_ROLL_POSITION, R_ELBOW_YAW_POSITION, L_HIP_ROLL_POSITION, L_HIP_PITCH_POSITION, L_HIP_YAWPITCH_POSITION, L_KNEE_PITCH_POSITION, L_ANKLE_ROLL_POSITION, L_ANKLE_PITCH_POSITION, R_HIP_ROLL_POSITION, R_HIP_PITCH_POSITION, R_HIP_YAWPITCH_POSITION, R_KNEE_PITCH_POSITION, R_ANKLE_ROLL_POSITION, R_ANKLE_PITCH_POSITION};
vector<string> NAOSensors::m_jointposition_names(temp_jointposition_names, temp_jointposition_names + sizeof(temp_jointposition_names)/sizeof(*temp_jointposition_names));

static string temp_jointtarget_names[] = {HEAD_PITCH_TARGET, HEAD_YAW_TARGET, L_SHOULDER_ROLL_TARGET, L_SHOULDER_PITCH_TARGET, L_ELBOW_ROLL_TARGET, L_ELBOW_YAW_TARGET, R_SHOULDER_ROLL_TARGET, R_SHOULDER_PITCH_TARGET, R_ELBOW_ROLL_TARGET, R_ELBOW_YAW_TARGET, L_HIP_ROLL_TARGET, L_HIP_PITCH_TARGET, L_HIP_YAWPITCH_TARGET, L_KNEE_PITCH_TARGET, L_ANKLE_ROLL_TARGET, L_ANKLE_PITCH_TARGET, R_HIP_ROLL_TARGET, R_HIP_PITCH_TARGET, R_HIP_YAWPITCH_TARGET, R_KNEE_PITCH_TARGET, R_ANKLE_ROLL_TARGET, R_ANKLE_PITCH_TARGET};
vector<string> NAOSensors::m_jointtarget_names(temp_jointtarget_names, temp_jointtarget_names + sizeof(temp_jointtarget_names)/sizeof(*temp_jointtarget_names));

static string temp_jointstiffness_names[] = {HEAD_PITCH_HARDNESS, HEAD_YAW_HARDNESS, L_SHOULDER_ROLL_HARDNESS, L_SHOULDER_PITCH_HARDNESS, L_ELBOW_ROLL_HARDNESS, L_ELBOW_YAW_HARDNESS, R_SHOULDER_ROLL_HARDNESS, R_SHOULDER_PITCH_HARDNESS, R_ELBOW_ROLL_HARDNESS, R_ELBOW_YAW_HARDNESS, L_HIP_ROLL_HARDNESS, L_HIP_PITCH_HARDNESS, L_HIP_YAWPITCH_HARDNESS, L_KNEE_PITCH_HARDNESS, L_ANKLE_ROLL_HARDNESS, L_ANKLE_PITCH_HARDNESS, R_HIP_ROLL_HARDNESS, R_HIP_PITCH_HARDNESS, R_HIP_YAWPITCH_HARDNESS, R_KNEE_PITCH_HARDNESS, R_ANKLE_ROLL_HARDNESS, R_ANKLE_PITCH_HARDNESS};
vector<string> NAOSensors::m_jointstiffness_names(temp_jointstiffness_names, temp_jointstiffness_names + sizeof(temp_jointstiffness_names)/sizeof(*temp_jointstiffness_names));

static string temp_jointcurrent_names[] = {HEAD_PITCH_CURRENT, HEAD_YAW_CURRENT, L_SHOULDER_ROLL_CURRENT, L_SHOULDER_PITCH_CURRENT, L_ELBOW_ROLL_CURRENT, L_ELBOW_YAW_CURRENT, R_SHOULDER_ROLL_CURRENT, R_SHOULDER_PITCH_CURRENT, R_ELBOW_ROLL_CURRENT, R_ELBOW_YAW_CURRENT, L_HIP_ROLL_CURRENT, L_HIP_PITCH_CURRENT, L_HIP_YAWPITCH_CURRENT, L_KNEE_PITCH_CURRENT, L_ANKLE_ROLL_CURRENT, L_ANKLE_PITCH_CURRENT, R_HIP_ROLL_CURRENT, R_HIP_PITCH_CURRENT, R_HIP_YAWPITCH_CURRENT, R_KNEE_PITCH_CURRENT, R_ANKLE_ROLL_CURRENT, R_ANKLE_PITCH_CURRENT};
vector<string> NAOSensors::m_jointcurrent_names(temp_jointcurrent_names, temp_jointcurrent_names + sizeof(temp_jointcurrent_names)/sizeof(*temp_jointcurrent_names));

static string temp_jointtemperature_names[] = {HEAD_PITCH_TEMPERATURE, HEAD_YAW_TEMPERATURE, L_SHOULDER_ROLL_TEMPERATURE, L_SHOULDER_PITCH_TEMPERATURE, L_ELBOW_ROLL_TEMPERATURE, L_ELBOW_YAW_TEMPERATURE, R_SHOULDER_ROLL_TEMPERATURE, R_SHOULDER_PITCH_TEMPERATURE, R_ELBOW_ROLL_TEMPERATURE, R_ELBOW_YAW_TEMPERATURE, L_HIP_ROLL_TEMPERATURE, L_HIP_PITCH_TEMPERATURE, L_HIP_YAWPITCH_TEMPERATURE, L_KNEE_PITCH_TEMPERATURE, L_ANKLE_ROLL_TEMPERATURE, L_ANKLE_PITCH_TEMPERATURE, R_HIP_ROLL_TEMPERATURE, R_HIP_PITCH_TEMPERATURE, R_HIP_YAWPITCH_TEMPERATURE, R_KNEE_PITCH_TEMPERATURE, R_ANKLE_ROLL_TEMPERATURE, R_ANKLE_PITCH_TEMPERATURE};
vector<string> NAOSensors::m_jointtemperature_names(temp_jointtemperature_names, temp_jointtemperature_names + sizeof(temp_jointtemperature_names)/sizeof(*temp_jointtemperature_names));

static string temp_accel_names[] = {ACCEL_X, ACCEL_Y, ACCEL_Z};
vector<string> NAOSensors::m_accel_names(temp_accel_names, temp_accel_names + sizeof(temp_accel_names)/sizeof(*temp_accel_names));

static string temp_gyro_names[] = {GYRO_X, GYRO_Y};
vector<string> NAOSensors::m_gyro_names(temp_gyro_names, temp_gyro_names + sizeof(temp_gyro_names)/sizeof(*temp_gyro_names));

static string temp_orientation_names[] = {ANGLE_X, ANGLE_Y};
vector<string> NAOSensors::m_orientation_names(temp_orientation_names, temp_orientation_names + sizeof(temp_orientation_names)/sizeof(*temp_orientation_names));

static string temp_foot_left_sole_names[] = {L_FSR_FL, L_FSR_FR, L_FSR_BR, L_FSR_BL};
static string temp_foot_right_sole_names[] = {R_FSR_FL, R_FSR_FR, R_FSR_BR, R_FSR_BL};
vector<string> NAOSensors::m_foot_left_sole_names(temp_foot_left_sole_names, temp_foot_left_sole_names + sizeof(temp_foot_left_sole_names)/sizeof(*temp_foot_left_sole_names));
vector<string> NAOSensors::m_foot_right_sole_names(temp_foot_right_sole_names, temp_foot_right_sole_names + sizeof(temp_foot_right_sole_names)/sizeof(*temp_foot_right_sole_names));

static string temp_foot_bumper_names[] = {L_BUMP_L, L_BUMP_R, R_BUMP_L, R_BUMP_R};
vector<string> NAOSensors::m_foot_bumper_names(temp_foot_bumper_names, temp_foot_bumper_names + sizeof(temp_foot_bumper_names)/sizeof(*temp_foot_bumper_names));

static string temp_button_names[] = {CHEST_BUTTON};
vector<string> NAOSensors::m_button_names(temp_button_names, temp_button_names + sizeof(temp_button_names)/sizeof(*temp_button_names));

static string temp_battery_names[] = {CHARGE, CURRENT, VOLTAGE_MIN, VOLTAGE_MAX, TEMPERATURE};
vector<string> NAOSensors::m_battery_names(temp_battery_names, temp_battery_names + sizeof(temp_battery_names)/sizeof(*temp_battery_names));

static string temp_ulstrasonic_left_distance[] = {	US_DISTANCE_LEFT_VALUE0, US_DISTANCE_LEFT_VALUE1, US_DISTANCE_LEFT_VALUE2, 
									US_DISTANCE_LEFT_VALUE3, US_DISTANCE_LEFT_VALUE4, US_DISTANCE_LEFT_VALUE5, 
									US_DISTANCE_LEFT_VALUE6, US_DISTANCE_LEFT_VALUE7, US_DISTANCE_LEFT_VALUE8, 
									US_DISTANCE_LEFT_VALUE9 	};
static string temp_ulstrasonic_right_distance[] = {	US_DISTANCE_RIGHT_VALUE0, US_DISTANCE_RIGHT_VALUE1, US_DISTANCE_RIGHT_VALUE2, 
									US_DISTANCE_RIGHT_VALUE3, US_DISTANCE_RIGHT_VALUE4, US_DISTANCE_RIGHT_VALUE5, 
									US_DISTANCE_RIGHT_VALUE6, US_DISTANCE_RIGHT_VALUE7, US_DISTANCE_RIGHT_VALUE8, 
									US_DISTANCE_RIGHT_VALUE9	};
vector<string> NAOSensors::m_ultrasonic_left_distances(temp_ulstrasonic_left_distance, temp_ulstrasonic_left_distance + sizeof(temp_ulstrasonic_left_distance)/sizeof(*temp_ulstrasonic_left_distance));
vector<string> NAOSensors::m_ultrasonic_right_distances(temp_ulstrasonic_right_distance, temp_ulstrasonic_right_distance + sizeof(temp_ulstrasonic_right_distance)/sizeof(*temp_ulstrasonic_right_distance));

/*! @brief Constructs a NUSensors for NAO class
 */
NAOSensors::NAOSensors()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NAOSensors::NAOSensors()" << endl;
#endif
    getSensorsFromALMemory();
    m_data->addSensors(m_jointposition_names);
    initBuffers();
    m_joint_ids = m_data->mapIdToIds(NUSensorsData::All);
}

/*! @brief Destructor
 */
NAOSensors::~NAOSensors()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NAOSensors::~NAOSensors()" << endl;
#endif
}

/*! @brief Gets the access to the sensors from Aldebaran
 
    That is I set up ALMemoryFastAccess connection with sensor values I require
 */
void NAOSensors::getSensorsFromALMemory()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NAOSensors::getSensorsFromALMemory()" << endl;
#endif
    m_al_positions_access = new ALMemoryFastAccess();
    m_al_positions_access->ConnectToVariables(NUNAO::m_broker, m_jointposition_names);
    m_al_targets_access = new ALMemoryFastAccess();
    m_al_targets_access->ConnectToVariables(NUNAO::m_broker, m_jointtarget_names);
    m_al_stiffness_access = new ALMemoryFastAccess();
    m_al_stiffness_access->ConnectToVariables(NUNAO::m_broker, m_jointstiffness_names);
    m_al_current_access = new ALMemoryFastAccess();
    m_al_current_access->ConnectToVariables(NUNAO::m_broker, m_jointcurrent_names);
    m_al_temperature_access = new ALMemoryFastAccess();
    m_al_temperature_access->ConnectToVariables(NUNAO::m_broker, m_jointtemperature_names);
    m_al_accel_access = new ALMemoryFastAccess();
    m_al_accel_access->ConnectToVariables(NUNAO::m_broker, m_accel_names);
    m_al_gyro_access = new ALMemoryFastAccess();
    m_al_gyro_access->ConnectToVariables(NUNAO::m_broker, m_gyro_names);
    m_al_orientation_access = new ALMemoryFastAccess();
    m_al_orientation_access->ConnectToVariables(NUNAO::m_broker, m_orientation_names);
    m_al_footleftsole_access = new ALMemoryFastAccess();
    m_al_footleftsole_access->ConnectToVariables(NUNAO::m_broker, m_foot_left_sole_names);
    m_al_footrightsole_access = new ALMemoryFastAccess();
    m_al_footrightsole_access->ConnectToVariables(NUNAO::m_broker, m_foot_right_sole_names);
    m_al_footbumper_access = new ALMemoryFastAccess();
    m_al_footbumper_access->ConnectToVariables(NUNAO::m_broker, m_foot_bumper_names);
    m_al_button_access = new ALMemoryFastAccess();
    m_al_button_access->ConnectToVariables(NUNAO::m_broker, m_button_names);
    m_al_battery_access = new ALMemoryFastAccess();
    m_al_battery_access->ConnectToVariables(NUNAO::m_broker, m_battery_names);
    m_al_ultrasonic_left_distances = new ALMemoryFastAccess();
    m_al_ultrasonic_left_distances->ConnectToVariables(NUNAO::m_broker, m_ultrasonic_left_distances);
    m_al_ultrasonic_right_distances = new ALMemoryFastAccess();
    m_al_ultrasonic_right_distances->ConnectToVariables(NUNAO::m_broker, m_ultrasonic_right_distances);
}

void NAOSensors::initBuffers()
{
    m_buffer_positions = vector<float>(m_jointposition_names.size(), 0);
    m_buffer_targets = vector<float>(m_jointposition_names.size(), 0);
    m_buffer_stiffnesses = vector<float>(m_jointposition_names.size(), 0);
    m_buffer_currents = vector<float>(m_jointposition_names.size(), 0);
    m_buffer_temperatures = vector<float>(m_jointposition_names.size(), 0);
    
    m_previous_positions = vector<float>(m_jointposition_names.size(), 0);
    m_previous_velocities = vector<float>(m_jointposition_names.size(), 0);
}

/*! @brief Copies the sensor data from almemory to NUSensorsData
 */
void NAOSensors::copyFromHardwareCommunications()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NAOSensors::copyFromHardwareCommunications()" << endl;
#endif
    // This took 0.13ms to complete in 2010
    // This takes 0.24ms to complete (28/11/2011: There is very little scope for improvement)
    
   	copyFromJoints();
    copyFromAccelerometerAndGyro();
    copyFromDistance();
    copyFromFeet();
    copyFromButtons();
    copyFromBattery();
}

void NAOSensors::copyFromJoints()
{
    static const float NaN = numeric_limits<float>::quiet_NaN();
    const id_t& pos_id = NUSensorsData::PositionId;
    const id_t& vel_id = NUSensorsData::VelocityId;
    const id_t& acc_id = NUSensorsData::AccelerationId;
    const id_t& tar_id = NUSensorsData::TargetId;
    const id_t& sti_id = NUSensorsData::StiffnessId;
    const id_t& cur_id = NUSensorsData::CurrentId;
    const id_t& tem_id = NUSensorsData::TemperatureId;
    
    m_al_positions_access->GetValues(m_buffer_positions);
    m_al_targets_access->GetValues(m_buffer_targets);
    m_al_stiffness_access->GetValues(m_buffer_stiffnesses);
    m_al_current_access->GetValues(m_buffer_currents);
    m_al_temperature_access->GetValues(m_buffer_temperatures);
    
    vector<float> joint(NUSensorsData::NumJointSensorIndices, NaN);
    float delta_t = 1000*(m_current_time - m_previous_time);
    for (size_t i=0; i<m_buffer_positions.size(); i++)
    {
        joint[pos_id] = m_buffer_positions[i];           
        joint[vel_id] = (joint[pos_id] - m_previous_positions[i])/delta_t;    
        joint[acc_id] = (joint[vel_id] - m_previous_velocities[i])/delta_t;
        joint[tar_id] = m_buffer_targets[i];       
        joint[sti_id] = 100*m_buffer_stiffnesses[i];        
        joint[cur_id] = m_buffer_currents[i];
        joint[tem_id] = m_buffer_temperatures[i];
        m_data->set(*m_joint_ids[i], m_current_time, joint);
        
        m_previous_positions[i] = joint[pos_id];
        m_previous_velocities[i] = joint[vel_id];
    }
}

void NAOSensors::copyFromAccelerometerAndGyro()
{    
    m_al_accel_access->GetValues(m_buffer_accelerometer);
    m_al_gyro_access->GetValues(m_buffer_gyrometer);
    m_al_orientation_access->GetValues(m_buffer_orientation);
    
    // The DCM stores the accelerometer values as raw hardware values
    // Thus, we need to convert to cm/s/s. According to the docs 63 units is approx equal to 981 cm/s/s; (981/63) = 15.571
    for (size_t i=0; i<m_buffer_accelerometer.size(); i++)
        m_buffer_accelerometer[i] = m_buffer_accelerometer[i]*15.571;             
    
    // The DCM stores the gyro values as raw hardware values
    // Thus we need to convert to rad/s. According to the forums 2.7 deg/s (PI/(2.7*180) = 1/154.7
    for (size_t i=0; i<m_buffer_gyrometer.size(); i++)      
        m_buffer_gyrometer[i] = m_buffer_gyrometer[i]/154.7;                    
    
    m_data->set(NUSensorsData::Accelerometer, m_current_time, m_buffer_accelerometer);
    m_data->set(NUSensorsData::Gyro, m_current_time, m_buffer_gyrometer); 
    m_data->set(NUSensorsData::OrientationHardware, m_current_time, m_buffer_orientation);   
}

void NAOSensors::copyFromDistance()
{
    m_al_ultrasonic_left_distances->GetValues(m_buffer_left_echos);
    m_al_ultrasonic_right_distances->GetValues(m_buffer_right_echos);
    
    // The DCM stores the distances in metres, so we need to convert to cm
    for(size_t i=0 ; i <m_buffer_left_echos.size(); i++)
    {
        m_buffer_left_echos[i] = m_buffer_left_echos[i]*100;
        m_buffer_right_echos[i] = m_buffer_right_echos[i]*100;
    }
    
    m_data->set(NUSensorsData::LDistance, m_current_time, m_buffer_left_echos);
    m_data->set(NUSensorsData::RDistance, m_current_time, m_buffer_right_echos);
}

void NAOSensors::copyFromFeet()
{
    m_al_footleftsole_access->GetValues(m_buffer_foot_lfsr);
    m_al_footrightsole_access->GetValues(m_buffer_foot_rfsr);
    m_al_footbumper_access->GetValues(m_buffer_foot_bumper);
    
    // The DCM stores the forces in kg, so we need to convert to Newtons
    for (size_t i=0; i<m_buffer_foot_lfsr.size(); i++)
    {
        m_buffer_foot_lfsr[i] = m_buffer_foot_lfsr[i]*9.81;
        m_buffer_foot_rfsr[i] = m_buffer_foot_rfsr[i]*9.81;
    }
    
    m_data->set(NUSensorsData::LFootTouch, m_current_time, m_buffer_foot_lfsr);
    m_data->set(NUSensorsData::RFootTouch, m_current_time, m_buffer_foot_rfsr);
    m_data->modify(NUSensorsData::LLegEndEffector, NUSensorsData::BumperId, m_current_time, m_buffer_foot_bumper[0] + m_buffer_foot_bumper[1]);
    m_data->modify(NUSensorsData::RLegEndEffector, NUSensorsData::BumperId, m_current_time, m_buffer_foot_bumper[2] + m_buffer_foot_bumper[3]);
}

void NAOSensors::copyFromButtons()
{
    m_al_button_access->GetValues(m_buffer_buttons);
    m_data->modify(NUSensorsData::MainButton, NUSensorsData::StateId, m_current_time, m_buffer_buttons[0]);
    m_data->modify(NUSensorsData::LeftButton, NUSensorsData::StateId, m_current_time, m_buffer_foot_bumper[0] + m_buffer_foot_bumper[1]);
    m_data->modify(NUSensorsData::RightButton, NUSensorsData::StateId, m_current_time, m_buffer_foot_bumper[2] + m_buffer_foot_bumper[3]);
}

void NAOSensors::copyFromBattery()
{
    m_al_battery_access->GetValues(m_buffer_battery);
    m_buffer_battery[2] = *(reinterpret_cast<int*>(&m_buffer_battery[2]))/1000.0;      // some casting madness for the battery values which are actually ints
    m_buffer_battery[3] = *(reinterpret_cast<int*>(&m_buffer_battery[3]))/1000.0;
    m_data->set(NUSensorsData::BatteryVoltage, m_current_time, 3*(m_buffer_battery[2] + m_buffer_battery[3]));  
    m_data->set(NUSensorsData::BatteryCurrent, m_current_time, m_buffer_battery[1]);
    m_data->set(NUSensorsData::BatteryCharge, m_current_time, m_buffer_battery[0]);
}



