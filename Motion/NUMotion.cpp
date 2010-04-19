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
#include "NUMotion.h"

#include "NUPlatform/NUPlatform.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "debug.h"
#include "debugverbositynumotion.h"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

/*! @brief Constructor for motion module
 */
NUMotion::NUMotion()
{
    m_current_time = 0;
    m_previous_time = 0;
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NUMotion::NUMotion" << endl;
#endif
#ifdef USE_HEAD
    m_head = new NUHead();
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

/*! @brief Process new sensor data, and produce actionator commands.
 
    This function basically calls all of the process functions of the submodules of motion. I have it setup
    so that the process functions are only called when they are allowed to be executed.
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. This variable will be filled
                   with the desired actions by NUMotion provided the NUActionatorsData instance
                   has been initialised by NUActionators.
 */
void NUMotion::process(NUSensorsData* data, NUActionatorsData* actions)
{
#if DEBUG_NUMOTION_VERBOSITY > 2
    debug << "NUMotion::process(" << data << ", " << actions << ")" << endl;
#endif
    if (data == NULL || actions == NULL)
        return;
    m_data = data;
    m_actions = actions;
    m_current_time = m_data->CurrentTime;
    calculateCycleTime();
    
    static vector<float> fallingvalues;
    static vector<float> fallenvalues;
    data->getFalling(fallingvalues);
    data->getFallen(fallenvalues);              //! @todo Put in a compile flag here or something because I need to walk while fallen atm
    if (false && fallingvalues[0] > 0)                           // If falling you can't do ANY motion except the fall protection.
        m_fall_protection->process(data, actions);
    else if (false && fallenvalues[0] > 0)                       // If fallen you can only getup
    {
        m_getup->process(data, actions);
        if (m_getup->headReady())                       // And you can only use the head if the getup lets you
        {
            #ifdef USE_HEAD
                m_head->process(data, actions);
            #endif
        }
    }
    else                                                // If not falling and not fallen I can do kicks, walks, saves and blocks
    {
        #ifdef USE_HEAD
            m_head->process(data, actions);
        #endif
        #ifdef USE_WALK
            m_walk->process(data, actions);
        #endif
        #ifdef USE_KICK
            m_kick->process(data, actions);
        #endif
    }
    m_previous_time = m_current_time;
}

/*! @brief Process the jobs. Jobs are deleted when they are completed, and more jobs can be added inside this function.
 
    @attention There is a very rare segmentation fault in this function. This will need to be looked at eventually.
               I think I might need to change things around so I don't remove jobs mid loop!
    
    @param jobs the current list of jobs
 */
void NUMotion::process(JobList& jobs)
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NUMotion::process():" << endl;
#endif
    
    static list<Job*>::iterator it;     // the iterator over the motion jobs
    for (it = jobs.motion_begin(); it != jobs.motion_end();)
    {
        #ifdef USE_WALK         // ---------------------------------------------------------- WalkJob processing
            if ((*it)->getID() == Job::MOTION_WALK)
            {   // process a walk speed job
                static vector<float> speed;
                static WalkJob* job;
                
                job = (WalkJob*) (*it);
                job->getSpeed(speed);
                #if DEBUG_NUMOTION_VERBOSITY > 4
                    debug << "NUMotion::process(): Processing a walkSpeed job." << endl;
                #endif
                
                m_walk->walkSpeed(speed);
                it = jobs.removeMotionJob(it);
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
                it = jobs.removeMotionJob(it);
            }
            else if ((*it)->getID() == Job::MOTION_WALK_PARAMETERS)
            {   // process a walk to point job
                static double time;
                static WalkParameters parameters;
                static WalkParametersJob* job;
                
                job = (WalkParametersJob*) (*it);
                job->getWalkParameters(parameters);
                #if DEBUG_NUMOTION_VERBOSITY > 4
                    debug << "NUMotion::process(): Processing a walkparameter job." << endl;
                    parameters.summaryTo(debug);
                #endif
                
                m_walk->setWalkParameters(parameters);
                it = jobs.removeMotionJob(it);
                #if DEBUG_NUMOTION_VERBOSITY > 4
                    debug << "NUMotion::process(): Processing a walkparameter job. After remove." << endl;
                #endif
            }
        #else
            if (false) {}       // need this here so it compiles when there is no walk
        #endif
        
        #ifdef USE_KICK         // ---------------------------------------------------------- KickJob processing
            else if ((*it)->getID() == Job::MOTION_KICK)
            {   // process a kick job
                static double time;
                static vector<float> kickposition;
                static vector<float> kicktarget;
                static KickJob* job;
                
                job = (KickJob*) (*it);
                job->getKick(time, kickposition, kicktarget);
                #if DEBUG_NUMOTION_VERBOSITY > 4
                    debug << "NUMotion::process(): Processing a kick job." << endl;
                #endif
                
                m_kick->kickToPoint(kickposition, kicktarget);
                it = jobs.removeMotionJob(it);
            }
        #endif
        
        #ifdef USE_HEAD         // ---------------------------------------------------------- HeadJob processing
            else if ((*it)->getID() == Job::MOTION_HEAD)
            {   // process a kick job
                static vector<double> times;
                static vector<vector<float> > headpositions;
                static HeadJob* job;
                
                job = (HeadJob*) (*it);
                job->getPositions(times, headpositions);
                #if DEBUG_NUMOTION_VERBOSITY > 4
                    debug << "NUMotion::process(): Processing a head job." << endl;
                #endif
                
                m_head->moveTo(times, headpositions);
                it = jobs.removeMotionJob(it);
            }
        #endif
            else 
            {
                ++it;
            }

    }
    #if DEBUG_NUMOTION_VERBOSITY > 4
        debug << "NUMotion::process(): Finished." << endl;
    #endif
}

/*! @brief Calculates the cycle time. 
 
 To be platform independent I calculate the motion cycle time online by averaging the cycle times
*/
void NUMotion::calculateCycleTime()
{
    using namespace boost::accumulators;
    static accumulator_set<float, stats<tag::mean> > cycle_time_accumulator;
    
    cycle_time_accumulator(m_current_time - m_previous_time);
    m_cycle_time = static_cast<int> (mean(cycle_time_accumulator));
    cout << m_cycle_time << endl;
}


