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
#include "NUPlatform/NUPlatform.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "Infrastructure/Jobs/JobList.h"
#include "debug.h"
#include "debugverbositynetwork.h"

using namespace std;

NUviewIO* nuio;

NUviewIO::NUviewIO(): NUIO(Blackboard->GameInfo, Blackboard->TeamInfo, Blackboard->Jobs)
{
#if DEBUG_NETWORK_VERBOSITY > 4
    debug << "NUviewIO::NUviewIO(" << static_cast<void*>(Blackboard->GameInfo) << ", " << static_cast<void*>(Blackboard->TeamInfo) << ", " << static_cast<void*>(Blackboard->Jobs) << ")" << endl;
#endif
    if (nuio == NULL)
        nuio = this;
}

NUviewIO::~NUviewIO()
{
#if DEBUG_NETWORK_VERBOSITY > 4
    debug << "NUviewIO::~NUviewIO()" << endl;
#endif
}

