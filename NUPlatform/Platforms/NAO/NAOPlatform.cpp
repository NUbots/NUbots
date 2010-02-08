/*! @file NAOPlatform.cpp
    @brief Implementation of NAOPlatform

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

#include "NAOPlatform.h"
#include "NAOCamera.h"
#include "NAOSensors.h"
#include "NAOActionators.h"
#include "NAOSystem.h"

#include "debug.h"

/*! @brief Constructor for NAO robotic platform
 */
NAOPlatform::NAOPlatform()
{
#if DEBUG_NUPLATFORM_VERBOSITY > 4
    debug << "NAOPlatform::NAOPlatform()" << endl;
#endif
    system = new NAOSystem();                 // the system needs to be created first because it provides times for the other modules!
    nusystem = system;                        // we access the system in other modules using this pointer.    
    camera = new NAOCamera();
    sensors = new NAOSensors();
    actionators = new NAOActionators();
#if DEBUG_NUPLATFORM_VERBOSITY > 4
    debug << "NAOPlatform::NAOPlatform(). Completed." << endl;
#endif
}

NAOPlatform::~NAOPlatform()
{
#if DEBUG_NUPLATFORM_VERBOSITY > 4
    debug << "NAOPlatform::~NAOPlatform()" << endl;
#endif
}

