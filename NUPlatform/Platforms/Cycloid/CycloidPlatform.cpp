/*! @file CycloidPlatform.cpp
    @brief Implementation of CycloidPlatform

    @author Jason Kulk
 
 Copyright (c) 2010 Jason Kulk
 
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

#include "CycloidPlatform.h"
#include "CycloidCamera.h"
#include "CycloidSensors.h"
#include "CycloidActionators.h"
#include "../Robotis/Motors.h"

#include "debug.h"
#include "debugverbositynuplatform.h"
#include "nubotconfig.h"

using namespace std;

/*! @brief Constructor for Bear robotic platform
 */
CycloidPlatform::CycloidPlatform()
{
#if DEBUG_NUPLATFORM_VERBOSITY > 4
    debug << "CycloidPlatform::CycloidPlatform" << endl;
#endif
    init();
    #ifdef USE_VISION
        m_camera = new CycloidCamera();
        #error CycloidCamera not implemented yet! Compile with USE_VISION set to OFF
    #else
        m_camera = 0;
    #endif
    m_motors = Motors::getInstance();
    m_sensors = new CycloidSensors(m_motors);
    m_actionators = new CycloidActionators(m_motors);
}

CycloidPlatform::~CycloidPlatform()
{
}

