/*! @file BearIO.cpp
    @brief Implementation of BearIO input/output class

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

#include "BearIO.h"
#include "NUbot.h"

#include "debug.h"
#include "debugverbositynetwork.h"
#include "ioconfig.h"



/*! @brief Construct a BearIO object
    @param nubot a pointer to the NUbot, we need this to gain access to the public store
 */
BearIO::BearIO(NUbot* nubot): NUIO(nubot)
{
#if DEBUG_NETWORK_VERBOSITY > 0
    debug << "BearIO::BearIO()" << std::endl;
#endif
    m_nubot = nubot;
}

BearIO::~BearIO()
{
#if DEBUG_NETWORK_VERBOSITY > 0
    debug << "BearIO::~BearIO()" << std::endl;
#endif
}

