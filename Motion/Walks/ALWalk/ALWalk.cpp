/*! @file JWalk.cpp
    @brief Implementation of jwalk class

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

#include "ALWalk.h"

#include "NUPlatform/NUSystem.h"
#include "NUPlatform/Platforms/NAO/NUNAO.h"
#include "debug.h"
#include "debugverbositynumotion.h"

#include <albroker.h>
#include <time.h>

ALWalk::ALWalk()
{   
    m_al_motion = new ALMotionProxy(NUNAO::m_broker);
}

/*! @brief Destructor for motion module
 */
ALWalk::~ALWalk()
{
    if (m_al_motion != NULL)
        delete m_al_motion;
}

/*! @brief Kill the aldebaran walk engine
 */
void ALWalk::kill()
{
    m_walk_enabled = false;
    m_al_motion->killWalk();
}

void ALWalk::doWalk()
{
    static unsigned int count = 0;
    static float max_x = 10.0;
    static float max_y = 2.0;
    static float max_yaw = 1.0;
    
    if (count%4 == 0)
    {   // this is a very simple hack to get almotion to use alot less CPU. It is perfectly reasonable to do this because almotion isn't going to respond that quickly anyway.
        if (fabs(m_speed_x) > max_x)
            m_speed_x = (m_speed_x/fabs(m_speed_x))*max_x;
        if (fabs(m_speed_y) > max_y)
            m_speed_y = (m_speed_y/fabs(m_speed_y))*max_y;
        if (fabs(m_speed_yaw) > max_yaw)
            m_speed_yaw = (m_speed_yaw/fabs(m_speed_yaw))*max_yaw;
        
        m_al_motion->setWalkTargetVelocity(m_speed_x/max_x, m_speed_y/max_y, m_speed_yaw/max_yaw, 1);
        count = 0;
    }
    count++;
    
    static vector<float> legnan(m_actions->getNumberOfJoints(NUActionatorsData::LeftLegJoints), NAN);
    static vector<float> armnan(m_actions->getNumberOfJoints(NUActionatorsData::LeftArmJoints), NAN);
    m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, m_data->CurrentTime, legnan, legnan, 75);
    m_actions->addJointPositions(NUActionatorsData::RightLegJoints, m_data->CurrentTime, legnan, legnan, 75);
    if (m_larm_enabled)
        m_actions->addJointPositions(NUActionatorsData::LeftArmJoints, m_data->CurrentTime, armnan, armnan, 30);
    if (m_rarm_enabled)
        m_actions->addJointPositions(NUActionatorsData::RightArmJoints, m_data->CurrentTime, armnan, armnan, 30);
}

/*! @brief Sets whether the arms are allowed to be moved by the walk engine
 */
void ALWalk::setArmEnabled(bool leftarm, bool rightarm)
{
    m_larm_enabled = leftarm;
    m_rarm_enabled = rightarm;
    m_al_motion->setWalkArmsEnable(leftarm, rightarm);
}
