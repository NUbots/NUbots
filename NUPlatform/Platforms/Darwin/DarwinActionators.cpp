/*! @file DarwinActionators.cpp
    @brief Implementation of darwin actionators class

    @author Jason Kulk
 
  Copyright (c) 2011 Jason Kulk
 
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

#include <iomanip>

#include "DarwinActionators.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "DarwinJointMapping.h"
#include "DarwinPlatform.h"
#include <cmath>

#include "debug.h"
#include "debugverbositynuactionators.h"
#include <limits>


static string temp_chestled_names[] = { "Chest/Led/"};
vector<string> DarwinActionators::m_chestled_names(temp_chestled_names, temp_chestled_names + sizeof(temp_chestled_names)/sizeof(*temp_chestled_names));
unsigned int DarwinActionators::m_num_chestleds = DarwinActionators::m_chestled_names.size();

static string temp_footled_names[] = {  "LFoot Led", "RFoot Led"};
vector<string> DarwinActionators::m_footled_names(temp_footled_names, temp_footled_names + sizeof(temp_footled_names)/sizeof(*temp_footled_names));
unsigned int DarwinActionators::m_num_footleds = DarwinActionators::m_footled_names.size();

DarwinActionators::DarwinActionators(DarwinPlatform* darwin, Robot::CM730* subboard)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "DarwinActionators::DarwinActionators()" <<endl;
    #endif
    m_current_time = 0;
    platform = darwin;
    cm730 = subboard;
    count = 0;
    vector<string> sound(1, "Sound");
    vector<string> names;
    names.insert(names.end(), platform->m_servo_names.begin(), platform->m_servo_names.end());
    names.insert(names.end(), m_chestled_names.begin(), m_chestled_names.end());
    names.insert(names.end(), m_footled_names.begin(), m_footled_names.end());
    names.insert(names.end(), sound.begin(), sound.end());
    m_data->addActionators(names);
    
    #if DEBUG_NUACTIONATORS_VERBOSITY > 0
        debug << "DarwinActionators::DarwinActionators(). Avaliable Actionators: " << endl;
        m_data->summaryTo(debug);
    #endif

    InitialiseMotors();
    m_joint_mapping = &DarwinJointMapping::Instance();

    // sensor_read_manager_ = subboard->sensor_read_manager();
}

DarwinActionators::~DarwinActionators()
{

}
void DarwinActionators::InitialiseMotors()
{
    for(int i = 0; i < platform->m_servo_IDs.size(); i++)
    {
        cm730->WriteByte(platform->m_servo_IDs[i], Robot::MX28::P_TORQUE_ENABLE, 0, 0);
    }
}

void DarwinActionators::copyToHardwareCommunications()
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 3
        debug << "DarwinActionators::copyToHardwareCommunications()" << endl;
    #endif
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        m_data->summaryTo(debug);
    #endif
    copyToServos();
    copyToLeds();
    copyToSound();
}


// int MapSensorIdToServoIndex(int sensor_id)
// {
//     switch(sensor_id)
//     {
//         case Robot::JointData::ID_HEAD_TILT       : return  0;
//         case Robot::JointData::ID_HEAD_PAN        : return  1;
//         case Robot::JointData::ID_L_SHOULDER_ROLL : return  2;
//         case Robot::JointData::ID_L_SHOULDER_PITCH: return  3;
//         case Robot::JointData::ID_L_ELBOW         : return  4;
//         case Robot::JointData::ID_R_SHOULDER_ROLL : return  5;
//         case Robot::JointData::ID_R_SHOULDER_PITCH: return  6;
//         case Robot::JointData::ID_R_ELBOW         : return  7;
//         case Robot::JointData::ID_L_HIP_ROLL      : return  8;
//         case Robot::JointData::ID_L_HIP_PITCH     : return  9;
//         case Robot::JointData::ID_L_HIP_YAW       : return 10;
//         case Robot::JointData::ID_L_KNEE          : return 11;
//         case Robot::JointData::ID_L_ANKLE_ROLL    : return 12;
//         case Robot::JointData::ID_L_ANKLE_PITCH   : return 13;
//         case Robot::JointData::ID_R_HIP_ROLL      : return 14;
//         case Robot::JointData::ID_R_HIP_PITCH     : return 15;
//         case Robot::JointData::ID_R_HIP_YAW       : return 16;
//         case Robot::JointData::ID_R_KNEE          : return 17;
//         case Robot::JointData::ID_R_ANKLE_ROLL    : return 18;
//         case Robot::JointData::ID_R_ANKLE_PITCH   : return 19;
//         default: {
//             std::cout   << __PRETTY_FUNCTION__
//                         << ": Invalid sensor_id: " << sensor_id << ";"
//                         << std::endl;
//             return -1;
//         }
//     }
// }


void DarwinActionators::copyToServos()
{
    static vector<float> positions;
    static vector<float> p_gains;
    
    // Get the values that must be written to the servos
    m_data->getNextServos(positions, p_gains);

    //Data for Sync Write:
    int sync_write_tx_packet[platform->m_servo_IDs.size() * (Robot::MX28::PARAM_BYTES)];
    int n = 0;
    int num_joints = 0;

    //Defaults from data sheet:
    // int P_GAIN = 64;
    int I_GAIN = 0;
    int D_GAIN = 0;

    // Build sync_write_tx_packet:
    for (size_t i = 0; i < platform->m_servo_IDs.size(); i++)
    {
        int sensor_index = i;
        int sensor_id = platform->m_servo_IDs[sensor_index];
    // std::vector<Robot::SensorReadDescriptor*> &sorted_descriptors = sensor_read_manager_->descriptor_heap_;
    // for (std::vector<Robot::SensorReadDescriptor*>::iterator it = sorted_descriptors.begin();
    //      it != sorted_descriptors.end(); ++it)
    // {
    //     Robot::SensorReadDescriptor* sensor_read = *it;
    //     int sensor_id = sensor_read->sensor_id();

    //     if(sensor_id > 20) continue;
    //     int sensor_index = MapSensorIdToServoIndex(sensor_id); // i; //
    //     if(sensor_index == -1) continue;

        // std::cout   << "Writing: "
        //             << std::setw(16) << Robot::SensorReadManager::SensorNameForId(sensor_id) 
        //             << " ("
        //             << std::setw(3) << sensor_id
        //             << "): index="
        //             << std::setw(3) << sensor_index
        //             << ", consecutive_errors:"
        //             // << std::setw(3) << sensor_read->consecutive_errors()
        //             << ";"
        //             << std::endl;

        platform->setMotorGoalPosition(sensor_index, positions[sensor_index]);
        // Note: 'setMotorStiffness' writes directly to the CM730 board,
        //       which is costly.
        //       Doing this for each motor sequentially is questionable.
        //       This code should be reviewed. -MM (2013-05-07)
        platform->setMotorStiffness(sensor_index, p_gains[sensor_index]);

        if(p_gains[sensor_index] > 0)
        {
            int value = m_joint_mapping->joint2rawClipped(sensor_index, positions[sensor_index]);
            sync_write_tx_packet[n++] = sensor_id;

            sync_write_tx_packet[n++] = D_GAIN;
            sync_write_tx_packet[n++] = I_GAIN;
            sync_write_tx_packet[n++] = (p_gains[sensor_index] * 100) / 128; // P_GAIN
            
            sync_write_tx_packet[n++] = 0;
            sync_write_tx_packet[n++] = Robot::CM730::GetLowByte(value);
            sync_write_tx_packet[n++] = Robot::CM730::GetHighByte(value);
            num_joints++;
        }
    }

    // Send new servo data to all motors at once:
    int error_code = cm730->SyncWrite(Robot::MX28::P_D_GAIN,
                                      Robot::MX28::PARAM_BYTES,
                                      num_joints,
                                      sync_write_tx_packet);
}

void DarwinActionators::copyToLeds()
{
    // LED DATA STRUCTURE:
    //  Chest:      LedValues[0][0][0,1,2]
    //  Feet Left:  LedValues[1][0][0,1,2]
    //  Feet Right: LedValues[2][0][0,1,2]

    // BOARD DATA STRUCTURE:
    //  LED:        [0 BBBB GGGG RRRR]
	
    if(count % 10 == 0)
    {
        static vector<  vector < vector < float > > > ledvalues;
        m_data->getNextLeds(ledvalues);
        int value = (int(ledvalues[0][0][0]*31) << 0) + (int(ledvalues[0][0][1]*31) << 5) + (int(ledvalues[0][0][2]*31) << 10);
        cm730->WriteWord(Robot::CM730::P_LED_HEAD_L, value, 0);

        if(count % 20 == 0)
        {
            int value = (int(ledvalues[1][0][0]*31) << 0) + (int(ledvalues[1][0][1]*31) << 5) + (int(ledvalues[1][0][2]*31) << 10);
            cm730->WriteWord(Robot::CM730::P_LED_EYE_L, value, 0);
        }
        else
        {
            int value = (int(ledvalues[2][0][0]*31) << 0) + (int(ledvalues[2][0][1]*31) << 5) + (int(ledvalues[2][0][2]*31) << 10);
            cm730->WriteWord(Robot::CM730::P_LED_EYE_L, value, 0);
        }
    }
    count++;
}



