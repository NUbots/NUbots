/*! @file NAOSensors.cpp
    @brief Implementation of NAO sensor class

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

#include "NAOSensors.h"
#include "NAOSensorNames.h"
#include "NUNAO.h"
#include "debug.h"
#include "debugverbositynusensors.h"

// Apparently the best way to initialise a vector like an array, is to initialise the vector from an array
static string temp_jointposition_names[] = {DN_HEAD_PITCH_POSITION, DN_HEAD_YAW_POSITION, DN_L_SHOULDER_ROLL_POSITION, DN_L_SHOULDER_PITCH_POSITION, DN_L_ELBOW_ROLL_POSITION, DN_L_ELBOW_YAW_POSITION, DN_R_SHOULDER_ROLL_POSITION, DN_R_SHOULDER_PITCH_POSITION, DN_R_ELBOW_ROLL_POSITION, DN_R_ELBOW_YAW_POSITION, DN_L_HIP_ROLL_POSITION, DN_L_HIP_PITCH_POSITION, DN_L_HIP_YAWPITCH_POSITION, DN_L_KNEE_PITCH_POSITION, DN_L_ANKLE_ROLL_POSITION, DN_L_ANKLE_PITCH_POSITION, DN_R_HIP_ROLL_POSITION, DN_R_HIP_PITCH_POSITION, DN_R_HIP_YAWPITCH_POSITION, DN_R_KNEE_PITCH_POSITION, DN_R_ANKLE_ROLL_POSITION, DN_R_ANKLE_PITCH_POSITION};
vector<string> NAOSensors::m_jointposition_names(temp_jointposition_names, temp_jointposition_names + sizeof(temp_jointposition_names)/sizeof(*temp_jointposition_names));

static string temp_jointtarget_names[] = {DN_HEAD_PITCH_TARGET, DN_HEAD_YAW_TARGET, DN_L_SHOULDER_ROLL_TARGET, DN_L_SHOULDER_PITCH_TARGET, DN_L_ELBOW_ROLL_TARGET, DN_L_ELBOW_YAW_TARGET, DN_R_SHOULDER_ROLL_TARGET, DN_R_SHOULDER_PITCH_TARGET, DN_R_ELBOW_ROLL_TARGET, DN_R_ELBOW_YAW_TARGET, DN_L_HIP_ROLL_TARGET, DN_L_HIP_PITCH_TARGET, DN_L_HIP_YAWPITCH_TARGET, DN_L_KNEE_PITCH_TARGET, DN_L_ANKLE_ROLL_TARGET, DN_L_ANKLE_PITCH_TARGET, DN_R_HIP_ROLL_TARGET, DN_R_HIP_PITCH_TARGET, DN_R_HIP_YAWPITCH_TARGET, DN_R_KNEE_PITCH_TARGET, DN_R_ANKLE_ROLL_TARGET, DN_R_ANKLE_PITCH_TARGET};
vector<string> NAOSensors::m_jointtarget_names(temp_jointtarget_names, temp_jointtarget_names + sizeof(temp_jointtarget_names)/sizeof(*temp_jointtarget_names));

static string temp_jointstiffness_names[] = {DN_HEAD_PITCH_HARDNESS, DN_HEAD_YAW_HARDNESS, DN_L_SHOULDER_ROLL_HARDNESS, DN_L_SHOULDER_PITCH_HARDNESS, DN_L_ELBOW_ROLL_HARDNESS, DN_L_ELBOW_YAW_HARDNESS, DN_R_SHOULDER_ROLL_HARDNESS, DN_R_SHOULDER_PITCH_HARDNESS, DN_R_ELBOW_ROLL_HARDNESS, DN_R_ELBOW_YAW_HARDNESS, DN_L_HIP_ROLL_HARDNESS, DN_L_HIP_PITCH_HARDNESS, DN_L_HIP_YAWPITCH_HARDNESS, DN_L_KNEE_PITCH_HARDNESS, DN_L_ANKLE_ROLL_HARDNESS, DN_L_ANKLE_PITCH_HARDNESS, DN_R_HIP_ROLL_HARDNESS, DN_R_HIP_PITCH_HARDNESS, DN_R_HIP_YAWPITCH_HARDNESS, DN_R_KNEE_PITCH_HARDNESS, DN_R_ANKLE_ROLL_HARDNESS, DN_R_ANKLE_PITCH_HARDNESS};
vector<string> NAOSensors::m_jointstiffness_names(temp_jointstiffness_names, temp_jointstiffness_names + sizeof(temp_jointstiffness_names)/sizeof(*temp_jointstiffness_names));

static string temp_jointcurrent_names[] = {DN_HEAD_PITCH_CURRENT, DN_HEAD_YAW_CURRENT, DN_L_SHOULDER_ROLL_CURRENT, DN_L_SHOULDER_PITCH_CURRENT, DN_L_ELBOW_ROLL_CURRENT, DN_L_ELBOW_YAW_CURRENT, DN_R_SHOULDER_ROLL_CURRENT, DN_R_SHOULDER_PITCH_CURRENT, DN_R_ELBOW_ROLL_CURRENT, DN_R_ELBOW_YAW_CURRENT, DN_L_HIP_ROLL_CURRENT, DN_L_HIP_PITCH_CURRENT, DN_L_HIP_YAWPITCH_CURRENT, DN_L_KNEE_PITCH_CURRENT, DN_L_ANKLE_ROLL_CURRENT, DN_L_ANKLE_PITCH_CURRENT, DN_R_HIP_ROLL_CURRENT, DN_R_HIP_PITCH_CURRENT, DN_R_HIP_YAWPITCH_CURRENT, DN_R_KNEE_PITCH_CURRENT, DN_R_ANKLE_ROLL_CURRENT, DN_R_ANKLE_PITCH_CURRENT};
vector<string> NAOSensors::m_jointcurrent_names(temp_jointcurrent_names, temp_jointcurrent_names + sizeof(temp_jointcurrent_names)/sizeof(*temp_jointcurrent_names));

static string temp_jointtemperature_names[] = {DN_HEAD_PITCH_TEMPERATURE, DN_HEAD_YAW_TEMPERATURE, DN_L_SHOULDER_ROLL_TEMPERATURE, DN_L_SHOULDER_PITCH_TEMPERATURE, DN_L_ELBOW_ROLL_TEMPERATURE, DN_L_ELBOW_YAW_TEMPERATURE, DN_R_SHOULDER_ROLL_TEMPERATURE, DN_R_SHOULDER_PITCH_TEMPERATURE, DN_R_ELBOW_ROLL_TEMPERATURE, DN_R_ELBOW_YAW_TEMPERATURE, DN_L_HIP_ROLL_TEMPERATURE, DN_L_HIP_PITCH_TEMPERATURE, DN_L_HIP_YAWPITCH_TEMPERATURE, DN_L_KNEE_PITCH_TEMPERATURE, DN_L_ANKLE_ROLL_TEMPERATURE, DN_L_ANKLE_PITCH_TEMPERATURE, DN_R_HIP_ROLL_TEMPERATURE, DN_R_HIP_PITCH_TEMPERATURE, DN_R_HIP_YAWPITCH_TEMPERATURE, DN_R_KNEE_PITCH_TEMPERATURE, DN_R_ANKLE_ROLL_TEMPERATURE, DN_R_ANKLE_PITCH_TEMPERATURE};
vector<string> NAOSensors::m_jointtemperature_names(temp_jointtemperature_names, temp_jointtemperature_names + sizeof(temp_jointtemperature_names)/sizeof(*temp_jointtemperature_names));

static string temp_accel_names[] = {DN_ACCEL_X, DN_ACCEL_Y, DN_ACCEL_Z};
vector<string> NAOSensors::m_accel_names(temp_accel_names, temp_accel_names + sizeof(temp_accel_names)/sizeof(*temp_accel_names));

static string temp_gyro_names[] = {DN_GYRO_X, DN_GYRO_Y};
vector<string> NAOSensors::m_gyro_names(temp_gyro_names, temp_gyro_names + sizeof(temp_gyro_names)/sizeof(*temp_gyro_names));

static string temp_foot_sole_names[] = {DN_L_FSR_FL, DN_L_FSR_FR, DN_L_FSR_BL, DN_L_FSR_BR, DN_R_FSR_FL, DN_R_FSR_FR, DN_R_FSR_BL, DN_R_FSR_BR};
vector<string> NAOSensors::m_foot_sole_names(temp_foot_sole_names, temp_foot_sole_names + sizeof(temp_foot_sole_names)/sizeof(*temp_foot_sole_names));

static string temp_foot_bumper_names[] = {DN_L_BUMP_L, DN_L_BUMP_R, DN_R_BUMP_L, DN_R_BUMP_R};
vector<string> NAOSensors::m_foot_bumper_names(temp_foot_bumper_names, temp_foot_bumper_names + sizeof(temp_foot_bumper_names)/sizeof(*temp_foot_bumper_names));

static string temp_button_names[] = {DN_CHEST_BUTTON};
vector<string> NAOSensors::m_button_names(temp_button_names, temp_button_names + sizeof(temp_button_names)/sizeof(*temp_button_names));

static string temp_battery_names[] = {DN_CHARGE, DN_CURRENT, DN_VOLTAGE_MIN, DN_VOLTAGE_MAX, DN_TEMPERATURE};
vector<string> NAOSensors::m_battery_names(temp_battery_names, temp_battery_names + sizeof(temp_battery_names)/sizeof(*temp_battery_names));

/*! @brief Constructs a NUSensors for NAO class
 */
NAOSensors::NAOSensors()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NAOSensors::NAOSensors()" << endl;
#endif
    getSensorsFromALMemory();
    m_data->setAvailableJoints(m_jointposition_names);
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
    m_al_footsole_access = new ALMemoryFastAccess();
    m_al_footsole_access->ConnectToVariables(NUNAO::m_broker, m_foot_sole_names);
    m_al_footbumper_access = new ALMemoryFastAccess();
    m_al_footbumper_access->ConnectToVariables(NUNAO::m_broker, m_foot_bumper_names);
    m_al_button_access = new ALMemoryFastAccess();
    m_al_button_access->ConnectToVariables(NUNAO::m_broker, m_button_names);
    m_al_battery_access = new ALMemoryFastAccess();
    m_al_battery_access->ConnectToVariables(NUNAO::m_broker, m_battery_names);
}

/*! @brief Copies the sensor data from almemory to NUSensorsData
 */
void NAOSensors::copyFromHardwareCommunications()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NAOSensors::copyFromHardwareCommunications()" << endl;
#endif
    static vector<float> temp;
    
    // This takes 0.13ms to complete
    m_al_positions_access->GetValues(temp);
    m_data->setJointPositions(m_current_time, temp);
    
    m_al_targets_access->GetValues(temp);
    m_data->setJointTargets(m_current_time, temp);
    
    m_al_stiffness_access->GetValues(temp);
    m_data->setJointStiffnesses(m_current_time, temp);
    
    m_al_current_access->GetValues(temp);
    m_data->setJointCurrents(m_current_time, temp);
    
    m_al_temperature_access->GetValues(temp);
    m_data->setJointTemperatures(m_current_time, temp);
    
    m_al_accel_access->GetValues(temp);
    for (unsigned int i=0; i<temp.size(); i++)      // we need to convert to cm/s/s
        temp[i] = temp[i]*15.571;                   // 63 units is approx equal to 981 cm/s/s; (981/63) = 15.571
    m_data->setBalanceAccelerometer(m_current_time, temp);
    
    m_al_gyro_access->GetValues(temp);
    for (unsigned int i=0; i<temp.size(); i++)      // we need to convert to rad/s
        temp[i] = temp[i]/154.7;                    // scaling factor: Alderbaran say it is 2.7 deg/s (PI/(2.7*180) = 1/154.7
    m_data->setBalanceGyro(m_current_time, temp);   
    
    m_al_footsole_access->GetValues(temp);
    for (unsigned int i=0; i<temp.size(); i++)
        temp[i] = temp[i]*9.81;
    m_data->setFootSoleValues(m_current_time, temp);
    
    m_al_footbumper_access->GetValues(temp);
    m_data->setFootBumperValues(m_current_time, temp);
    
    m_al_button_access->GetValues(temp);
    m_data->setButtonValues(m_current_time, temp);
    
    m_al_battery_access->GetValues(temp);
    m_data->setBatteryValues(m_current_time, temp);    
}




