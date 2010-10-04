/*! @file BearPlatform.cpp
    @brief Implementation of BearPlatform

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

#include "BearPlatform.h"
#include "BearCamera.h"
#include "BearSensors.h"
#include "BearActionators.h"
#include "Serial/Motors.h"
#include "BearSystem.h"

#include "debug.h"
#include "debugverbositynuplatform.h"
#include "nubotconfig.h"

using namespace std;

/*! @brief Constructor for Bear robotic platform
 */
BearPlatform::BearPlatform()
{
#if DEBUG_NUPLATFORM_VERBOSITY > 4
    debug << "BearPlatform::BearPlatform" << endl;
#endif
    
    system = new BearSystem();                 // the system needs to be created first because it provides times for the other modules!
    #ifdef USE_VISION
        camera = new BearCamera();
        #error BearCamera not implemented yet! Compile with USE_VISION set to OFF
    #else
        camera = 0;
    #endif
    motors = Motors::getInstance();
    sensors = new BearSensors(motors);
    actionators = new BearActionators(motors);

    motors->torqueOn(Motors::IndexToMotorID, MOTORS_NUM_MOTORS);
}

BearPlatform::~BearPlatform()
{
}

