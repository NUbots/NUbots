/*! @file NUMotion.cpp
    @brief Implementation of motion class

    @author Jason Kulk
 
 So what can Motion do?
    - walk
    - track points, pan, nod with the head
    - do kicks
    - play scripts (get-ups, and probably blocks)
 
 So that looks like four sub modules.
 
 NUMotion gets some jobs; BODY and HEAD. Now I need to convert those jobs into
 other jobs, or put the actions in the NUActionatorsData
 
 WALK.
    Input:
        NUSensorsData
        Current walk-related job
    Output:
        NUActionatorsData
 
 HEAD.
    Input: 
        NUSensorsData (I think that you should proably try to stablise the head, and maybe use some smarter control)
        Current head-related job
    Output:
        NUActionatorsData
 
 KICK.
    Input:
        NUSensorsData (The kick should be closed loop)
        Current kick-related job
    Output:
        NUActionatorsData
 
 BLOCK.
 
 SAVE.
 
 GETUP.
 
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

#include <iostream>
using namespace std;

#include "NUPlatform/NUPlatform.h"
#include "NUMotion.h"
#include "Tools/debug.h"

/*! @brief Constructor for motion module
 */
NUMotion::NUMotion()
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NUMotion::NUMotion" << endl;
#endif
}

/*! @brief Destructor for motion module
 */
NUMotion::~NUMotion()
{
}

/*! @brief Process new sensor data, and produce actionator commands
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. This variable will be filled
                   with the desired actions by NUMotion provided the NUActionatorsData instance
                   has been initialised by NUActionators.
 */
void NUMotion::process(NUSensorsData* data, NUActionatorsData* actions)
{
    if (actions == NULL)
        return;
    
}

/*! @brief Process jobs
 */
void NUMotion::process(JobList jobs)
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NUMotion::process():" << endl;
#endif
    
    // 
    
    // I need to easily iterate over the job list
    // for each job in joblist:
    //      if job.type == BODY:
    //          if job.id == STAND:
    //              m_walk->walkToPoint(job.x, job.y, job.theta);
    //          elif job.id == WALK:
    //              m_walk->walkOnVector(job.x, job.y, job.theta);
    
    // Option 1. NUMotion does the organisation.
    //           if iCanKickFromHere(job.x, job.y, job.theta, job.targetx, job.targety)
    //              m_kick->kickPoint(job.x, job.y, job.theta, job.targetx, job.targety)         // This means kick might have to call walk's stop
    //           else:
    //              m_walk->walkToPoint(m_kick->getNearestPoint(job.x, job.y, job.theta, job.targetx, job.targety));
    
    // Option 2. KICK etc does the organisation
    //          elif job.id == KICK:
    //              m_kick->kick(job.x, job.y, job.targetx, job.targety)                        // This means kick gets to call as many walk functions as it likes
    //              so kick could return a walkjob and nuactionatordata
    //              so the walkjob could be a stand(0,0) for a stop

    
    //          elif (job.id == BLOCK || job.id == SAVE):
    //              m_walk->walkToPoint(job.x, job.y, job.theta);
    //              m_walk->blockPoint(job.targetx, job.targety, usehands == false || true)
    //
}

