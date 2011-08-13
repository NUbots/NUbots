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

#include "DarwinActionators.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include <cmath>

#include "debug.h"
#include "debugverbositynuactionators.h"

/*! @brief Constructs a nubot actionator class with a Darwin backend
            
           The Darwin backend takes aspects from both the NAO and the Robotis backends; the Darwin
           has a secondary board to perform all the communication with hardware, like the NAO. However,
           it also uses Robotis motors, like the Cycloid/Bear.
 
           This backend is also the most recent, and probably should serve as a template for future platforms.
 */ 
DarwinActionators::DarwinActionators(DarwinPlatform* darwin,Robot::CM730* subboard)
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << "DarwinActionators::DarwinActionators()" << endl;
    #endif
    m_current_time = 0;
    
	platform = darwin;
	cm730 = subboard;

    vector<string> sound(1, "Sound");
	vector<string> names;
    names.insert(names.end(), platform->m_servo_names.begin(), platform->m_servo_names.end());
    names.insert(names.end(), sound.begin(), sound.end());
    m_data->addActionators(names);
	
    m_data->addActionators(platform->m_servo_names);
    
    #if DEBUG_NUACTIONATORS_VERBOSITY > 0
        debug << "DarwinActionators::DarwinActionators(). Avaliable Actionators: " << endl;
        m_data->summaryTo(debug);
    #endif
}

DarwinActionators::~DarwinActionators()
{
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
	
    for (size_t i=0; i<positions.size(); i++)
    {
		//cm730->WriteByte(m_servo_IDs[i],Robot::MX28::P_P_GAIN, 1, 0);
    	//cm730->WriteWord(m_servo_IDs[i],Robot::MX28::P_TORQUE_ENABLE, 1, 0);
		if(gains[i] > 0)
		{
			int value = Radian2Value(positions[i]-platform->m_servo_Offsets[i]);
			cm730->WriteWord(platform->m_servo_IDs[i],Robot::MX28::P_TORQUE_ENABLE, 1, 0);
			cm730->WriteWord(platform->m_servo_IDs[i],Robot::MX28::P_GOAL_POSITION_L,value,0);
		}
		else
		{
			cm730->WriteWord(platform->m_servo_IDs[i],Robot::MX28::P_TORQUE_ENABLE, 0, 0);
		}
	}
	

	/*//Test Head Only:
	for (size_t i=0; i<2; i++)
    {
		int value = Radian2Value(positions[i]-platform->m_servo_Offsets[i]);
		platform->cm730->WriteWord(platform->m_servo_IDs[i],Robot::MX28::P_GOAL_POSITION_L,value,0);
		platform->cm730->WriteByte(platform->m_servo_IDs[i],Robot::MX28::P_P_GAIN, 1, 0);
    	platform->cm730->WriteWord(platform->m_servo_IDs[i],Robot::MX28::P_TORQUE_ENABLE, 1, 0);
	}*/
}

void DarwinActionators::copyToLeds()
{
    
}



