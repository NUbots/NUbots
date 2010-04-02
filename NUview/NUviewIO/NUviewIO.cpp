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

#include "NUviewIO.h"
#include "NUPlatform/NUSystem.h"
#include "GameController/GameInformation.h"
#include "Behaviour/TeamInformation.h"
#include "Behaviour/Jobs/JobList.h"
#include "debug.h"
#include "debugverbositynetwork.h"

using namespace std;

NUviewIO* nuio;
NUSystem* NUviewIO::m_nusystem = new NUSystem();
GameInformation* NUviewIO::m_gameinfo = new GameInformation(1,1);
TeamInformation* NUviewIO::m_teaminfo = new TeamInformation();
JobList* NUviewIO::m_jobs = new JobList();

NUviewIO::NUviewIO(): NUIO(m_gameinfo, m_teaminfo, m_jobs)
{
#if DEBUG_NETWORK_VERBOSITY > 4
    debug << "NUviewIO::NUviewIO(" << static_cast<void*>(m_gameinfo) << ", " << static_cast<void*>(m_teaminfo) << ", " << static_cast<void*>(m_jobs) << ")" << endl;
#endif
    if (nuio == NULL)
        nuio = this;
}

NUviewIO::~NUviewIO()
{
#if DEBUG_NETWORK_VERBOSITY > 4
    debug << "NUviewIO::~NUviewIO()" << endl;
#endif
    if (m_nusystem != NULL)
        delete m_nusystem;
}

