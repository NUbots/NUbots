/*! @file CycloidIO.cpp
    @brief Implementation of CycloidIO input/output class

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

#include "CycloidIO.h"
#include "NUbot.h"

#include "debug.h"
#include "debugverbositynetwork.h"
#include "ioconfig.h"

using namespace std;

/*! @brief Construct a CycloidIO object
    @param nubot a pointer to the NUbot, we need this to gain access to the public store
 */
CycloidIO::CycloidIO(NUbot* nubot): NUIO(nubot)
{
#if DEBUG_NETWORK_VERBOSITY > 0
    debug << "CycloidIO::CycloidIO()" << endl;
#endif
    m_nubot = nubot;
}

CycloidIO::~CycloidIO()
{
#if DEBUG_NETWORK_VERBOSITY > 0
    debug << "CycloidIO::~CycloidIO()" << endl;
#endif
}

