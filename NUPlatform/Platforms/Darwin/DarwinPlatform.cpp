/*! @file DarwinPlatform.cpp
    @brief Implementation of DarwinPlatform

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

#include "DarwinPlatform.h"
#include "DarwinCamera.h"
#include "DarwinSensors.h"
#include "DarwinActionators.h"

#include "debug.h"
#include "debugverbositynuplatform.h"
#include "nubotconfig.h"

using namespace std;

/*! @brief Constructor for Bear robotic platform
 */
DarwinPlatform::DarwinPlatform()
{
#if DEBUG_NUPLATFORM_VERBOSITY > 4
    debug << "DarwinPlatform::DarwinPlatform" << endl;
#endif
    init();
    #ifdef USE_VISION
        m_camera = new DarwinCamera();
    #else
        m_camera = 0;
    #endif
    m_sensors = new DarwinSensors();
    m_actionators = new DarwinActionators();
}

DarwinPlatform::~DarwinPlatform()
{
}

