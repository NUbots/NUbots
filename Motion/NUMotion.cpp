/*! @file NUMotion.cpp
    @brief Implementation of motion class

    @author Jason Kulk
 
 Copyright (c) 2009, 2010 Jason Kulk
 
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
#ifdef USE_HEAD
    #include "NUHead.h"
#endif
#ifdef USE_WALK
    #include "NUWalk.h"
#endif
#ifdef USE_KICK
    #include "NUKick.h"
#endif
#if defined(USE_BLOCK) or defined(USE_SAVE)
    #include "NUSave.h"
#endif
#ifdef USE_SCRIPT
    #include "Script.h"
#endif

#include "Behaviour/Jobs.h"
#include "FallProtection.h"
#include "Getup.h"
#include "Tools/MotionScript.h"

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
    #if DEBUG_NUMOTION_VERBOSITY > 4
        debug << "NUMotion::NUMotion" << endl;
    #endif
    m_current_time = 0;
    m_previous_time = 0;
    m_last_kill_time = m_current_time - 10000;
    
    m_data = NULL;
    m_actions = NULL;
    #ifdef USE_HEAD
        m_head = new NUHead();
    #endif
    
    #if defined(USE_WALK)
        m_walk = NUWalk::getWalkEngine();
        #if defined(USE_KICK)
            m_kick = new NUKick(m_walk);
        #endif
        #if defined(USE_BLOCK) or defined(USE_SAVE)
            m_save = new NUSave(m_walk);
        #endif
        #if defined(USE_SCRIPT)
            m_script = new Script(m_walk);
        #endif
    #else
        #if defined(USE_KICK)
            m_kick = new NUKick(NULL);
        #endif
        #if defined(USE_BLOCK) or defined(USE_SAVE)
            m_save = new NUSave(NULL);
        #endif
        #if defined(USE_SCRIPT)
            m_script = new Script(NULL);
        #endif
    #endif
    
    //m_block_left = new MotionScript("BlockLeft");
}

/*! @brief Destructor for motion module
 */
NUMotion::~NUMotion()
{
    if (m_fall_protection != NULL)
        delete m_fall_protection;
    
    if (m_getup != NULL)
        delete m_getup;                   
    #ifdef USE_HEAD
        if (m_head != NULL)
            delete m_head;
    #endif
    #ifdef USE_WALK
        if (m_walk != NULL)
            delete m_walk;                
    #endif
    #ifdef USE_KICK
        if (m_kick != NULL)
            delete m_kick;                   
    #endif
    #if defined(USE_BLOCK) or defined(USE_SAVE)
        delete m_save;
    #endif
    #ifdef USE_SCRIPT
        delete m_script;
    #endif
}

/*! @brief Freezes all motion modules
 */
void NUMotion::freeze()
{
    m_last_kill_time = m_current_time;
    #ifdef USE_HEAD
        m_head->kill();
    #endif
    #ifdef USE_WALK
        m_walk->kill();                
    #endif
    #ifdef USE_KICK
        m_kick->kill();                   
    #endif
    #if defined(USE_BLOCK) or defined(USE_SAVE)
        m_save->kill();
    #endif
}

/*! @brief Adds actions to bring the robot to rest quickly, and go into a safe-for-robot pose
 */
void NUMotion::kill()
{
    freeze();
    float safelegpositions[] = {0, -0.85, -0.15, 2.16, 0, -1.22};
    float safelarmpositions[] = {0, 1.41, -1.1, -0.65};
    float saferarmpositions[] = {0, 1.41, 1.1, 0.65};
    vector<float> legpositions(safelegpositions, safelegpositions + sizeof(safelegpositions)/sizeof(*safelegpositions));
    vector<float> larmpositions(safelarmpositions, safelarmpositions + sizeof(safelarmpositions)/sizeof(*safelarmpositions));
    vector<float> rarmpositions(saferarmpositions, saferarmpositions + sizeof(saferarmpositions)/sizeof(*saferarmpositions));
    vector<float> legvelocities(legpositions.size(), 1.0);
    vector<float> armvelocities(larmpositions.size(), 1.0);
    
    // check if there is a reason it is not safe or possible to go into the crouch position
    if (m_actions == NULL)
        return;
    else if (m_data != NULL)
    {
        vector<float> orientation;
        if (m_data->getOrientation(orientation))
            if (fabs(orientation[0]) > 0.5 or fabs(orientation[1]) > 0.5)
                return;
        if (not m_data->isOnGround())
                return;
    }
    
    // go into safe mode
    m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime() + 1250, legpositions, legvelocities, 50);
    m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime() + 1250, legpositions, legvelocities, 50);
    m_actions->addJointPositions(NUActionatorsData::LeftArmJoints, nusystem->getTime() + 500, larmpositions, armvelocities, 30);
    m_actions->addJointPositions(NUActionatorsData::RightArmJoints, nusystem->getTime() + 500, rarmpositions, armvelocities, 30);
    
    m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, nusystem->getTime() + 2000, legpositions, legvelocities, 0);
    m_actions->addJointPositions(NUActionatorsData::RightLegJoints, nusystem->getTime() + 2000, legpositions, legvelocities, 0);
    m_actions->addJointPositions(NUActionatorsData::LeftArmJoints, nusystem->getTime() + 2000, larmpositions, armvelocities, 0);
    m_actions->addJointPositions(NUActionatorsData::RightArmJoints, nusystem->getTime() + 2000, rarmpositions, armvelocities, 0);
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
    
    if (m_fall_protection->enabled() and m_data->isFalling())
    {   // if falling no other motion module can run
        m_fall_protection->process(data, actions);
    }
    else if (m_getup->enabled() and (m_data->isFallen() or m_getup->isActive()))
    {   // if fallen over or getting up then only getup can run, and the head if getup has finished with it
        m_getup->process(data, actions);
        if (not m_getup->isUsingHead())
        {
            #ifdef USE_HEAD
                m_head->process(data, actions);
            #endif
        }
    }
    else
    {   // if we aren't falling, fallen or getting up then we can run some of the other motion modules
        #ifdef USE_HEAD
            m_head->process(data, actions);
        #endif
        
        // if kick or save are running then they must run until completion (unless interrupted by fall protection or getup)
        #ifdef USE_KICK
        if (m_kick->isActive())
            m_kick->process(data, actions);
        else
        #endif
        #if defined(USE_BLOCK) or defined(USE_SAVE)
            if (m_save->isActive())
                m_save->process(data, actions);
            else {
        #endif
            // if kick and save aren't running then we can run scripts and walk
            #ifdef USE_SCRIPT
            if (m_script->isActive())
                m_script->process(data, actions);
            if (not m_script->isUsingLegs())
            #endif
                #ifdef USE_WALK
                    m_walk->process(data, actions);
                #else
                    ;
                #endif
        #if defined(USE_KICK) or defined(USE_BLOCK) or defined(USE_SAVE)
            }
        #endif
    }
    
    m_previous_time = m_current_time;
    
    /*static bool alreadyran = false;
    if (m_current_time > 15500 and not alreadyran)
    {
        m_block_left->play(data, actions);
        alreadyran = true;
    }*/
}

/*! @brief Process the jobs. Jobs are deleted when they are completed, and more jobs can be added inside this function.
    
    @param jobs the current list of jobs
 */
void NUMotion::process(JobList* jobs)
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NUMotion::process(): Start" << endl;
#endif
    if (jobs == NULL || m_current_time < m_last_kill_time + 5000)
        return;
    
    list<Job*>::iterator it = jobs->motion_begin();     // the iterator over the motion jobs
    while (it != jobs->motion_end())
    {
        Job::job_id_t id = (*it)->getID();
        switch (id) 
        {
        #ifdef USE_WALK
            case Job::MOTION_WALK:
                m_walk->process(reinterpret_cast<WalkJob*> (*it));
                break;
            case Job::MOTION_WALK_TO_POINT:
                m_walk->process(reinterpret_cast<WalkToPointJob*> (*it));
                break;
            case Job::MOTION_WALK_PARAMETERS:
                m_walk->process(reinterpret_cast<WalkParametersJob*> (*it));
                break;
        #endif
        #ifdef USE_KICK
            case Job::MOTION_KICK:
                m_kick->process(reinterpret_cast<KickJob*> (*it));
                break;
        #endif
        #ifdef USE_HEAD
            case Job::MOTION_HEAD:
                m_head->process(reinterpret_cast<HeadJob*> (*it));
                break;
            case Job::MOTION_TRACK:
                m_head->process(reinterpret_cast<HeadTrackJob*> (*it));
                break;
            case Job:: MOTION_PAN:
                m_head->process(reinterpret_cast<HeadPanJob*> (*it));
                break;
            case Job::MOTION_NOD:
                m_head->process(reinterpret_cast<HeadNodJob*> (*it));
                break;
        #endif
        #if defined(USE_BLOCK) or defined(USE_SAVE)
            case Job::MOTION_BLOCK:
                m_save->process(reinterpret_cast<BlockJob*> (*it));
                break;
            case Job::MOTION_SAVE:
                m_save->process(reinterpret_cast<SaveJob*> (*it));
                break;
        #endif
        #ifdef USE_SCRIPT
            case Job::MOTION_SCRIPT:
                m_script->process(reinterpret_cast<ScriptJob*> (*it));
                break;
        #endif
            case Job::MOTION_KILL:
                process(reinterpret_cast<MotionKillJob*> (*it));
                break;
            case Job::MOTION_FREEZE:
                process(reinterpret_cast<MotionFreezeJob*> (*it));
                break;
            default:
                break;
        }
        it = jobs->removeMotionJob(it);
    }
    
    #if DEBUG_NUMOTION_VERBOSITY > 4
        debug << "NUMotion::process(): Finished" << endl;
    #endif
}

void NUMotion::process(MotionKillJob* job)
{
    kill();
}

void NUMotion::process(MotionFreezeJob* job)
{
    freeze();
}

