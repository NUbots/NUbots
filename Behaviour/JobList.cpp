/*! @file JobList.cpp
    @brief Implementation of JobList class

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

#include "JobList.h"
#include "Tools/debug.h"

/*! @brief JobList constructor
 */
JobList::JobList()
{
    m_job_lists.push_back(&m_vision_jobs);
    m_job_lists.push_back(&m_localisation_jobs);
    m_job_lists.push_back(&m_behaviour_jobs);
    m_job_lists.push_back(&m_motion_jobs);
    m_job_lists.push_back(&m_light_jobs);
    m_job_lists.push_back(&m_camera_jobs);
    m_job_lists.push_back(&m_sound_jobs);
    m_job_lists.push_back(&m_system_jobs);
    m_job_lists.push_back(&m_other_jobs);
}

/*! @brief Job destructor
 */
JobList::~JobList()
{
}

/*! @brief Add a job to the job list. The type inside the job will be used to determine what type it is ;)
    @param job the job to be added
 */
void JobList::addJob(Job* job)
{
    if (job == NULL)
        return;
    Job::job_type_t jobtype = job->getJobType();
    if (jobtype == Job::MOTION)
        addMotionJob(job);
    else if (jobtype == Job::VISION)
        addVisionJob(job);
    else if (jobtype == Job::LOCALISATION)
        addLocalisationJob(job);
    else if (jobtype == Job::BEHAVIOUR)
        addBehaviourJob(job);
    else if (jobtype == Job::LIGHT)
        addLightJob(job);
    else if (jobtype == Job::CAMERA)
        addCameraJob(job);
    else if (jobtype == Job::SOUND)
        addSoundJob(job);
    else if (jobtype == Job::SYSTEM)
        addSystemJob(job);
    else if (jobtype == Job::OTHER)
        addOtherJob(job);
    else
        debug << "JobList::addJob. Unknown job type. Your job will not be added." << endl;
}

/*! @brief Add a vision job to the list
    @param job the job to be added
 */
void JobList::addVisionJob(Job* job)
{
    addJob(job, m_vision_jobs);
}

/*! @brief Add a localisation job to the list
    @param job the job to be added
 */
void JobList::addLocalisationJob(Job* job)
{
    addJob(job, m_localisation_jobs);
}

/*! @brief Add a behaviour job to the list
    @param job the job to be added
 */
void JobList::addBehaviourJob(Job* job)
{
    addJob(job, m_behaviour_jobs);
}

/*! @brief Add a motion job to the list
    @param job the job to be added
 */ 
void JobList::addMotionJob(Job* job)
{
    addJob(job, m_motion_jobs);
}

/*! @brief Add a light job to the list
    @param job the job to be added
 */
void JobList::addLightJob(Job* job)
{
    addJob(job, m_light_jobs);
}

/*! @brief Add a camera job to the list
    @param job the job to be added
 */
void JobList::addCameraJob(Job* job)
{
    addJob(job, m_camera_jobs);
}

/*! @brief Add a sound job to the list
    @param job the job to be added
 */
void JobList::addSoundJob(Job* job)
{
    addJob(job, m_sound_jobs);
}

/*! @brief Add a system job to the list
    @param job the job to be added
 */
void JobList::addSystemJob(Job* job)
{
    addJob(job, m_system_jobs);
}

/*! @brief Add a other job to the list
    @param job the job to be added
 */
void JobList::addOtherJob(Job* job)
{
    addJob(job, m_other_jobs);
}

/*! @brief Adds a job to the passed in list
    @param job the job to be added to joblist
    @param joblist the list to which job is added
 */
void JobList::addJob(Job* job, list<Job*>& joblist)
{
    joblist.push_back(job);
}

/*! @brief Remove a job from the list
    @param job the job to be removed
 */
void JobList::removeJob(Job* job)
{
    if (job == NULL)
        return;
    Job::job_type_t jobtype = job->getJobType();
    if (jobtype == Job::MOTION)
        removeMotionJob(job);
    else if (jobtype == Job::VISION)
        removeVisionJob(job);
    else if (jobtype == Job::LOCALISATION)
        removeLocalisationJob(job);
    else if (jobtype == Job::BEHAVIOUR)
        removeBehaviourJob(job);
    else if (jobtype == Job::LIGHT)
        removeLightJob(job);
    else if (jobtype == Job::CAMERA)
        removeCameraJob(job);
    else if (jobtype == Job::SOUND)
        removeSoundJob(job);
    else if (jobtype == Job::SYSTEM)
        removeSystemJob(job);
    else if (jobtype == Job::OTHER)
        removeOtherJob(job);
    else
        debug << "JobList::removeJob. Unknown job type. Your job was never added to begin with." << endl;
}

/*! @brief Remove a vision job from the list
    @param job the job to be removed
 */
void JobList::removeVisionJob(Job* job)
{
    removeJob(job, m_vision_jobs);
}

/*! @brief Remove a localisation job from the list
    @param job the job to be removed
 */
void JobList::removeLocalisationJob(Job* job)
{
    removeJob(job, m_localisation_jobs);
}

/*! @brief Remove a behaviour job from the list
    @param job the job to be removed
 */
void JobList::removeBehaviourJob(Job* job)
{
    removeJob(job, m_behaviour_jobs);
}

/*! @brief Remove a motion job from the list
    @param job the job to be removed
 */
void JobList::removeMotionJob(Job* job)
{
    removeJob(job, m_motion_jobs);
}

/*! @brief Remove a light job from the list
    @param job the job to be removed
 */
void JobList::removeLightJob(Job* job)
{
    removeJob(job, m_light_jobs);
}

/*! @brief Remove a camera job from the list
    @param job the job to be removed
 */
void JobList::removeCameraJob(Job* job)
{
    removeJob(job, m_camera_jobs);
}

/*! @brief Remove a sound job from the list
    @param job the job to be removed
 */
void JobList::removeSoundJob(Job* job)
{
    removeJob(job, m_sound_jobs);
}

/*! @brief Remove a system job from the list
    @param job the job to be removed
 */
void JobList::removeSystemJob(Job* job)
{
    removeJob(job, m_system_jobs);
}

/*! @brief Remove a other job from the list
    @param job the job to be removed
 */
void JobList::removeOtherJob(Job* job)
{
    removeJob(job, m_other_jobs);
}

/*! @brief Remove a job from the passed in list
    @param job the job to be removed
    @param joblist the list from which the job will be removed
 */
void JobList::removeJob(Job* job, list<Job*>& joblist)
{
    joblist.remove(job);
}


void JobList::clear()
{
    list<list<Job*>*>::iterator it;
    for (it = m_job_lists.begin(); it != m_job_lists.end(); it++)
        (*it)->clear();
}



