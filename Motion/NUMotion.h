/*! @file NUMotion.h
    @brief Declaration of motion class
 
    @class NUMotion
    @brief A module to provide motion
 
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

#ifndef NUMOTION_H
#define NUMOTION_H

#include "motionconfig.h"
#include "Behaviour/Jobs.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "NUWalk.h"
#include "NUHead.h"
#include "FallProtection.h"
#include "Getup.h"

class NUMotion
{
public:
    NUMotion();
    ~NUMotion();
    
    void process(NUSensorsData* data, NUActionatorsData* actions);
    void process(JobList& jobs);
protected:
private:
public:
protected:
public:         //! @todo TODO: Fix this. Jason needs a backdoor to the walk engine NOW!
    // essential motion components
    FallProtection* m_fall_protection;
    Getup* m_getup;
#ifdef USE_HEAD
    NUHead* m_head;
#endif
#ifdef USE_WALK
    NUWalk* m_walk;
#endif
#ifdef USE_KICK
    NUKick* m_kick;
#endif
};

#endif

