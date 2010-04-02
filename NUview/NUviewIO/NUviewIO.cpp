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

#include "NUViewIO.h"
#include "NUPlatform/NUSystem.h"
#include "debug.h"
#include "debugverbositynetwork.h"

using namespace std;

NUViewIO* nuio;

NUViewIO::NUViewIO(GameInformation* gameinfo, TeamInformation* teaminfo, JobList* jobs): NUIO(gameinfo, teaminfo, jobs)
{
#if DEBUG_NETWORK_VERBOSITY > 4
    debug << "NUViewIO::NUViewIO(" << static_cast<void*>(gameinfo) << ", " << static_cast<void*>(teaminfo) << ", " << static_cast<void*>(jobs) << ")" << endl;
#endif
    m_nusystem = new NUSystem();
    if (nuio == NULL)
        nuio = this;
}

NUViewIO::~NUViewIO()
{
#if DEBUG_NETWORK_VERBOSITY > 4
    debug << "NUViewIO::~NUViewIO()" << endl;
#endif
    if (m_nusystem != NULL)
        delete m_nusystem;
}

