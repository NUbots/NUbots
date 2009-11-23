/*! @file NAOWebotsSensors.cpp
    @brief Implementation of NAO in Webots sensor class

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

#include "NAOWebotsSensors.h"
#include "Tools/debug.h"

#include "webots/Servo.hpp"
using namespace webots;

/*! @brief Constructs a nubot sensor class with Webots backend
 */
NAOWebotsSensors::NAOWebotsSensors(NAOWebotsPlatform* platform)
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NAOWebotsSensors::NAOWebotsSensors()" << endl;
#endif
    Servo* hy = platform->getServo("HeadFail");
    debug << Servo::exists("HeadFail") << endl;
}

/*! @brief Destructor for NAOWebotsSensors
 */
NAOWebotsSensors::~NAOWebotsSensors()
{
}

/*! @brief Gets the sensor data using the Webots API and puts it in the NUSensorsData data member.
 */
void NAOWebotsSensors::copyFromHardwareCommunications()
{
#if DEBUG_NUSENSORS_VERBOSITY > 4
    debug << "NAOWebotsSensors::copyFromHardwareCommunications()" << endl;
#endif
    data.JointPositions;
}





