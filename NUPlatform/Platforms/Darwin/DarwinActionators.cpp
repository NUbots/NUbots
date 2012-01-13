/*! @file DarwinActionators.cpp
    @brief Implementation of darwin actionators class

    @author Jason Kulk, Steven Nicklin
 
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

#include "DarwinActionators.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "DarwinJointMapping.h"
#include "DarwinPlatform.h"

#include <cmath>

#include "debug.h"
#include "debugverbositynuactionators.h"
#include <limits>


/*! @brief Constructs a nubot actionator class with a Darwin backend
            
           The Darwin backend takes aspects from both the NAO and the Robotis backends; the Darwin
           has a secondary board to perform all the communication with hardware, like the NAO. However,
           it also uses Robotis motors, like the Cycloid/Bear.
 
           This backend is also the most recent, and probably should serve as a template for future platforms.
 */ 

static string temp_chestled_names[] = { "Chest/Led/"};
vector<string> DarwinActionators::m_chestled_names(temp_chestled_names, temp_chestled_names + sizeof(temp_chestled_names)/sizeof(*temp_chestled_names));
unsigned int DarwinActionators::m_num_chestleds = DarwinActionators::m_chestled_names.size();

static string temp_footled_names[] = {  "LFoot Led", "RFoot Led"};
vector<string> DarwinActionators::m_footled_names(temp_footled_names, temp_footled_names + sizeof(temp_footled_names)/sizeof(*temp_footled_names));
unsigned int DarwinActionators::m_num_footleds = DarwinActionators::m_footled_names.size();

DarwinActionators::DarwinActionators(DarwinPlatform* darwin,Robot::CM730* subboard)
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

void DarwinActionators::copyToServos()
{
    static vector<float> positions;
    static vector<float> gains;
    
    m_data->getNextServos(positions, gains);

    //Data for Sync Write:
    int param[platform->m_servo_IDs.size() * (Robot::MX28::PARAM_BYTES)];
    int n = 0;
    int joint_num = 0;

    //Defaults from data sheet:
    // int P_GAIN = 64;
    int I_GAIN = 0;
    int D_GAIN = 0;
	
    for (size_t i=0; i < platform->m_servo_IDs.size(); i++)
    {
        platform->setMotorGoalPosition(i,positions[i]);
        platform->setMotorStiffness(i,gains[i]);
		
        //cm730->WriteByte(m_servo_IDs[i],Robot::MX28::P_P_GAIN, 1, 0);
    	//cm730->WriteWord(m_servo_IDs[i],Robot::MX28::P_TORQUE_ENABLE, 1, 0);
        /*
        if(gains[i] > 0)
        {
            int value = Radian2Value(positions[i]-platform->m_servo_Offsets[i]);
            //int value = Radian2Value(0-platform->m_servo_Offsets[i]);
            cm730->WriteWord(platform->m_servo_IDs[i],Robot::MX28::P_TORQUE_ENABLE, 1, 0);
            cm730->WriteWord(platform->m_servo_IDs[i],Robot::MX28::P_GOAL_POSITION_L,value,0);
        }
        else
        {
            //cm730->WriteWord(platform->m_servo_IDs[i],Robot::MX28::P_TORQUE_ENABLE, 0, 0);
        }
        */

        if(gains[i] > 0)
        {
            int value = m_joint_mapping->joint2raw(i, positions[i]);
            param[n++] = platform->m_servo_IDs[i];
            //param[n++] = P_GAIN;
            param[n++] = gains[i] / 128 * 100;
            param[n++] = I_GAIN;
            param[n++] = D_GAIN;
            param[n++] = 0;
            param[n++] = Robot::CM730::GetLowByte(value);
            param[n++] = Robot::CM730::GetHighByte(value);
            joint_num++;
        }
    }
    int result = cm730->SyncWrite(Robot::MX28::P_P_GAIN, Robot::MX28::PARAM_BYTES, joint_num, param);
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



