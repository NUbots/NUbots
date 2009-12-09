/*! @file JobList.h
    @brief Declaration of JobList class.
 
    @class JobList
    @brief A class containing the list of jobs to be done by modules
 

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

#ifndef JOBLIST_H
#define JOBLIST_H

#include "Job.h"

#include <list>
#include <iterator>
using namespace std;

class JobList
{
public:
    JobList();
    ~JobList();
    
    void addJob(Job* job);
    void addVisionJob(Job* job);
    void addLocalisationJob(Job* job);
    void addBehaviourJob(Job* job);
    void addMotionJob(Job* job);
    void addLightJob(Job* job);
    void addCameraJob(Job* job);
    void addSoundJob(Job* job);
    void addSystemJob(Job* job);
    void addOtherJob(Job* job);
    
    void removeJob(Job* job);
    void removeVisionJob(Job* job);
    void removeLocalisationJob(Job* job);
    void removeBehaviourJob(Job* job);
    void removeMotionJob(Job* job);
    void removeLightJob(Job* job);
    void removeCameraJob(Job* job);
    void removeSoundJob(Job* job);
    void removeSystemJob(Job* job);
    void removeOtherJob(Job* job);
    
    void clear();
    
private:
    void addJob(Job* job, list<Job*>& joblist);
    void removeJob(Job* job, list<Job*>& joblist);

private:
    list<Job*> m_vision_jobs;               //!< a list of all the current vision jobs
    list<Job*> m_localisation_jobs;         //!< a list of all the current localisation jobs
    list<Job*> m_behaviour_jobs;            //!< a list of all the behaviour jobs
    list<Job*> m_motion_jobs;               //!< a list of all the current motion jobs
    list<Job*> m_light_jobs;                //!< a list of all the current light jobs
    list<Job*> m_camera_jobs;               //!< a list of all the current camera jobs
    list<Job*> m_sound_jobs;                //!< a list of all the current jobs for the sound system
    list<Job*> m_system_jobs;               //!< a list of all the current system/os jobs
    list<Job*> m_other_jobs;                //!< a list of all other jobs
    list<list<Job*>* > m_job_lists;         //!< a list of all the lists of jobs
};


// I think I need to implement an iterator!
class JobListIterator : public iterator<forward_iterator_tag, Job*>
{
};

#endif

