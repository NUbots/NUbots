/*! @file NUHead.cpp
    @brief Implementation of nuhead class

    @author Jed Rietveld
 
 Copyright (c) 2010 Jed Rietveld
 
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


#include "NUHead.h"
#include "NUPlatform/NUSystem.h"
#include "debug.h"
#include "debugverbositynumotion.h"


NUHead::NUHead()
{
    m_pitch = 0;
    m_yaw = 0;
}

/*! @brief Destructor for motion module
 */
NUHead::~NUHead()
{

}

/*! @brief Process new sensor data, and produce actionator commands
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. 
*/
void NUHead::process(NUSensorsData* data, NUActionatorsData* actions)
{
    if (actions == NULL || data == NULL)
        return;
    m_data = data;
    m_actions = actions;
    doHead();
}


void NUHead::process(const vector<float>& position)
{
	m_pitch = position[0];
	m_yaw = position[1];
	
	m_head_timestamp = nusystem->getTime();


}

void NUHead::doHead()
{
	static vector<float> pos (2,0);
	static vector<float> vel (2,0);
	static vector<float> gain (2,45);

	pos[0] = m_pitch;
	pos[1] = m_yaw;

	m_actions->addJointPositions(NUActionatorsData::HeadJoints, m_data->CurrentTime, pos, vel, gain);
}

