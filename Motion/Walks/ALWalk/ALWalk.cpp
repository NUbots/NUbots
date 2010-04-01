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
    debug << "ALWalk::ALWalk()" << endl;
    m_al_motion = new ALMotionProxy(NUNAO::m_broker);

    m_al_motion->setStiffnesses(string("Body"), 1.0f);
    m_al_motion->setWalkTargetVelocity(0.0, 0, 0, 0);
}

/*! @brief Destructor for motion module
 */
ALWalk::~ALWalk()
{
    if (m_al_motion != NULL)
        delete m_al_motion;
}

void ALWalk::doWalk()
{
    static unsigned int count = 0;
    if (count%4 == 0)
    {   // this is a very simple hack to get almotion to use alot less CPU. It is perfectly reasonable to do this because almotion isn't going to respond that quickly anyway.
        if (fabs(m_speed_x) > 10)
            m_speed_x = (m_speed_x/fabs(m_speed_x))*10;
        if (fabs(m_speed_y) > 10)
            m_speed_y = (m_speed_y/fabs(m_speed_y))*10;
        if (fabs(m_speed_yaw) > 1)
            m_speed_yaw = (m_speed_yaw/fabs(m_speed_yaw));
        
        m_al_motion->setWalkTargetVelocity(m_speed_x/10.0, m_speed_y/10.0, m_speed_yaw, 1);
    }
}


