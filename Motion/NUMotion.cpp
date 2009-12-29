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
#ifdef USE_HEAD
    m_head = ?;
#endif
#ifdef USE_WALK
    m_walk = NUWalk::getWalkEngine();       // I'd really like the switching between walk engines to be done at another level!
#endif
#ifdef USE_KICK
    m_kick = new NUKick();
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
#ifdef USE_WALK
    m_walk->process(data, actions);
#endif
}

/*! @brief Process the jobs
    
    @param jobs the current list of jobs
 */
void NUMotion::process(JobList& jobs)
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NUMotion::process():" << endl;
#endif
    
    static list<Job*>::iterator it;     // the iterator over the motion jobs
    for (it = jobs.motion_begin(); it != jobs.motion_end(); ++it)
    {
#ifdef USE_WALK
        if ((*it)->getID() == Job::MOTION_WALK)
        {   // process a walk speed job
            static vector<float> speed;
            static WalkJob* job;
            
            job = (WalkJob*) (*it);
            job->getSpeed(speed);         // why does it not know what type of job it is here?
            #if DEBUG_NUMOTION_VERBOSITY > 4
                debug << "NUMotion::process(): Processing a walkSpeed job." << endl;
            #endif
            
            m_walk->walkSpeed(speed);
        }
        else if ((*it)->getID() == Job::MOTION_WALK_TO_POINT)
        {   // process a walk to point job
            static double time;
            static vector<float> position;
            static WalkToPointJob* job;
            
            job = (WalkToPointJob*) (*it);
            job->getPosition(time, position);
            #if DEBUG_NUMOTION_VERBOSITY > 4
                debug << "NUMotion::process(): Processing a walkToPoint job." << endl;
            #endif
            
            m_walk->walkToPoint(time, position);
        }
#endif  // USE_WALK
        
#ifdef USE_KICK
        if ((*it)->getID() == Job::MOTION_KICK)
        {   // process a kick job
            static double time;
            static vector<float> kickposition;
            static vector<float> kicktarget;
            static KickJob* job;
            
            job = (KickJob*) (*it);
            job->getKick(time, kickposition, kicktarget);         // why does it not know what type of job it is here?
#if DEBUG_NUMOTION_VERBOSITY > 4
            debug << "NUMotion::process(): Processing a walkSpeed job." << endl;
#endif
            
            m_kick->kick(time, kickposition, kicktarget);
        }
#endif  // USE_KICK 
    }
}

