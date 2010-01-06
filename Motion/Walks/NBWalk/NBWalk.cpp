/*! @file NBWalk.cpp
    @brief Implementation of NBWalk class

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

#include "NBWalk.h"

#include "NUPlatform/NUSystem.h"
#include "Tools/debug.h"

#include <math.h>

//! @todo TODO: put M_PI and NORMALISE somewhere else
#ifndef M_PI
    #define M_PI 3.1415926535
#endif

template <class T>
inline T NORMALISE(T theta){
    return atan2(sin(theta), cos(theta));
}

NBWalk::NBWalk()
{
}

/*! @brief Destructor for motion module
 */
NBWalk::~NBWalk()
{
}

void NBWalk::doWalk()
{
    debug << "NBWalk::doWalk()" << endl;
}



