/*! @file NAOIO.cpp
    @brief Implementation of NAOIO input/output class

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

#include "NAOWebotsIO.h"
#include "debug.h"
#include "debugverbositynetwork.h"

#include "NUbot.h"

NAOWebotsIO::NAOWebotsIO(int probotnumber, NUbot* nubot) : NUIO(probotnumber, nubot)
{
#if DEBUG_NETWORK_VERBOSITY > 4
    debug << "NAOWebotsIO::NAOWebotsIO()" << endl;
#endif
}

NAOWebotsIO::~NAOWebotsIO()
{
#if DEBUG_NETWORK_VERBOSITY > 4
    debug << "NAOWebotsIO::~NAOWebotsIO()" << endl;
#endif
}

