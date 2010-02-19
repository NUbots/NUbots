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
#include "debug.h"

#include "../Jobs.h"            //!< @todo remove this after finished testing!

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
    
    
    // Test Save Job
    vector<float> saveposition(3, 0);
    saveposition[0] = 10;
    saveposition[1] = -25;
    saveposition[2] = 1.57;
    SaveJob* savejob = new SaveJob(300.1, saveposition);
    BlockJob* blockjob = new BlockJob(69, saveposition);
    HeadJob* headjob = new HeadJob(9000, saveposition);
    WalkToPointJob* pointjob = new WalkToPointJob(33, saveposition);
    // Test Kick Job
    vector<float> kickposition(2, 0);
    vector<float> kicktarget(2, 0);
    kickposition[0] = 0;
    kickposition[1] = -5.7;
    kicktarget[0] = 330.33;
    kicktarget[1] = 55.5;
    KickJob* kickjob = new KickJob(1010.19, kickposition, kicktarget);
    NodHeadJob* nodjob = new NodHeadJob(123, kicktarget, kickposition);
    PanHeadJob* panjob = new PanHeadJob(0.0000123, kicktarget, kickposition);
    // Test Walk Job
    vector<float> walkspeed(3, 0);
    walkspeed[0] = 10;
    walkspeed[1] = -25;
    walkspeed[2] = 0.0;
    WalkJob* walkjob = new WalkJob(walkspeed);
    WalkParameters parameters = WalkParameters();
    ifstream testparafile("jupptestparameters.wp");
    testparafile >> parameters;
    WalkParametersJob* parametersjob = new WalkParametersJob(parameters);
    
    /*// Test Light Job
    vector<float> colour(3,0);
    colour[0] = 1;
    colour[1] = 0;
    colour[2] = 0;
    ChestLedJob ledjob = ChestLedJob(0, colour);*/
    
    addMotionJob(savejob);
    addMotionJob(blockjob);
    addMotionJob(headjob);
    addMotionJob(kickjob);
    addMotionJob(nodjob);
    addMotionJob(panjob);
    addMotionJob(walkjob);
    addMotionJob(pointjob);
    addMotionJob(parametersjob);
    
    /*ofstream tempjoblog;
    tempjoblog.open("testjobs.txt");
    tempjoblog << (*this);
    tempjoblog.close();*/
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
    Job::job_type_t jobtype = job->getType();
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
    Job::job_type_t jobtype = job->getType();
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

/*! @brief Returns an iterator at the beginning of the job list. This iterator goes over the
           entire contents of the list.
 
    This iterator is fairly expensive, and should be avoided. Its expensive because finding non-empty
    lists and consequently valid jobs is expensive, and every call to .begin() or .end() *searches* for the
    appropriate element!
 */
JobList::iterator JobList::begin()
{
    return JobList::iterator(this);
}

/*! @brief Returns an iterator at the end of the job list. This iterator goes over the
           entire contents of the list.
 
    This iterator is fairly expensive, and should be avoided. Its expensive because finding non-empty
    lists and consequently valid jobs is expensive, and every call to .begin() or .end() *searches* for the
    appropriate element!
 */
JobList::iterator JobList::end()
{
    return JobList::iterator(this, true);
}

/*! @brief Returns an iterator at the beginning of the vision jobs.
 */
list<Job*>::iterator JobList::vision_begin()
{
    return m_vision_jobs.begin();
}

/*! @brief Returns an iterator at the end of the vision jobs.
 */
list<Job*>::iterator JobList::vision_end()
{
    return m_vision_jobs.end();
}

/*! @brief Returns an iterator at the beginning of the localisation jobs.
 */
list<Job*>::iterator JobList::localisation_begin()
{
    return m_localisation_jobs.begin();
}

/*! @brief Returns an iterator at the end of the localisation jobs.
 */
list<Job*>::iterator JobList::localisation_end()
{
    return m_localisation_jobs.end();
}

/*! @brief Returns an iterator at the beginning of the behaviour jobs.
 */
list<Job*>::iterator JobList::behaviour_begin()
{
    return m_behaviour_jobs.begin();
}

/*! @brief Returns an iterator at the end of the behaviour jobs.
 */
list<Job*>::iterator JobList::behaviour_end()
{
    return m_behaviour_jobs.end();
}

/*! @brief Returns an iterator at the beginning of the motion jobs.
 */
list<Job*>::iterator JobList::motion_begin()
{
    return m_motion_jobs.begin();
}

/*! @brief Returns an iterator at the end of the motion jobs.
 */
list<Job*>::iterator JobList::motion_end()
{
    return m_motion_jobs.end();
}

/*! @brief Returns an iterator at the beginning of the light jobs.
 */
list<Job*>::iterator JobList::light_begin()
{
    return m_light_jobs.begin();
}

/*! @brief Returns an iterator at the end of the light jobs.
 */
list<Job*>::iterator JobList::light_end()
{
    return m_light_jobs.end();
}

/*! @brief Returns an iterator at the beginning of the camera jobs.
 */
list<Job*>::iterator JobList::camera_begin()
{
    return m_camera_jobs.begin();
}

/*! @brief Returns an iterator at the end of the camera jobs.
 */
list<Job*>::iterator JobList::camera_end()
{
    return m_camera_jobs.end();
}

/*! @brief Returns an iterator at the beginning of the sound jobs.
 */
list<Job*>::iterator JobList::sound_begin()
{
    return m_sound_jobs.begin();
}

/*! @brief Returns an iterator at the end of the sound jobs.
 */
list<Job*>::iterator JobList::sound_end()
{
    return m_sound_jobs.end();
}

/*! @brief Returns an iterator at the beginning of the system jobs.
 */
list<Job*>::iterator JobList::system_begin()
{
    return m_system_jobs.begin();
}

/*! @brief Returns an iterator at the end of the system jobs.
 */
list<Job*>::iterator JobList::system_end()
{
    return m_system_jobs.end();
}

/*! @brief Returns an iterator at the beginning of the other jobs.
 */
list<Job*>::iterator JobList::other_begin()
{
    return m_other_jobs.begin();
}

/*! @brief Returns an iterator at the end of the other jobs.
 */
list<Job*>::iterator JobList::other_end()
{
    return m_other_jobs.end();
}

/*! @brief Clears the contents of the job list
 */
void JobList::clear()
{
    list<list<Job*>*>::iterator it;
    for (it = m_job_lists.begin(); it != m_job_lists.end(); it++)
        (*it)->clear();
}

/*! @brief Returns the size of the job list
    @return Returns the size of the job list 
 */
unsigned int JobList::size()
{
    unsigned int size = 0;
    list<list<Job*>*>::iterator it;
    for (it = m_job_lists.begin(); it != m_job_lists.end(); it++)
        size += (*it)->size();
    return size;
}

void JobList::summaryTo(ostream& output)
{
    output << "JobList " << size() << " -----------------------------------" << endl;
    static JobList::iterator it;     // the iterator over all of the jobs
    for (it = begin(); it != end(); ++it)
        (*it)->summaryTo(output);
    output << "-------------------------------------------" << endl;
}

void JobList::csvTo(ostream& output)
{
    static JobList::iterator it;     // the iterator over all of the jobs
    for (it = begin(); it != end(); ++it)
        (*it)->csvTo(output);
}

ostream& operator<<(ostream& output, JobList& joblist)
{
#if DEBUG_BEHAVIOUR_VERBOSITY > 4
    debug << "<<JobList" << endl;
#endif
    output << joblist.size() << " ";
    static JobList::iterator it;     // the iterator over all of the jobs
    for (it = joblist.begin(); it != joblist.end(); ++it)
        output << *it;
    return output;
}

istream& operator>>(istream& input, JobList& joblist)
{
#if DEBUG_BEHAVIOUR_VERBOSITY > 4
    debug << ">>JobList" << endl;
#endif
    unsigned int numnewjobs = 0;
    input >> numnewjobs;
    Job* tempjob = NULL;
    for (unsigned int i=0; i<numnewjobs; i++)
    {
        input >> &tempjob;
        joblist.addJob(tempjob);
    }
    return input;
}



/******************************************************************************************************************************************
                                                                                                            JobListIterator Implementation
 ******************************************************************************************************************************************/
/*! @brief Default constructor
 */
JobListIterator::JobListIterator()
{
    m_job = NULL;
}

/*! @brief Constructor for a JobListIterator over joblist
    @param joblist the JobList to iterate over
    @param end set this to false if you want the iterator at the beginning, set it to true if you want the iterator at the end
 */
JobListIterator::JobListIterator(JobList* joblist, bool end)
{
    m_joblist = joblist;
#if DEBUG_BEHAVIOUR_VERBOSITY > 5
    debug << "JobListIterator::JobListIterator. Contents of JobList:" << endl;
    list<list<Job*>*>::iterator it;
    list<Job*>::iterator sit;
    for (it = m_joblist->m_job_lists.begin(); it != m_joblist->m_job_lists.end(); ++it)
    {
        for (sit = (*it)->begin(); sit!=(*it)->end(); ++sit)
            debug << (*sit) << " ";
    }
    debug << endl;
#endif
    
    if (end == false)
    {   // make the iterator point to the beginning of the JobList
        // find the first non-empty list, but iterating over joblist->m_job_lists
        for (m_job_lists_iterator = m_joblist->m_job_lists.begin(); m_job_lists_iterator!=m_joblist->m_job_lists.end(); ++m_job_lists_iterator)
        {
            if (!(*m_job_lists_iterator)->empty())
            {
                m_list_iterator = (*m_job_lists_iterator)->begin();
                m_job = *m_list_iterator;
                break;
            }
        }
    }
    else
    {   // make the iterator point to the end of the JobList
        // find the last non-empty list, by iterating over joblist->m_job_lists backwards
        list<list<Job*>*>::reverse_iterator reverseit;
        for (reverseit = joblist->m_job_lists.rbegin(); reverseit != joblist->m_job_lists.rend(); ++reverseit)
        {
            if (!(*reverseit)->empty())
            {
                m_list_iterator = (*reverseit)->end();
                m_job = *m_list_iterator;
                break;
            }
        }
    }
    
    // this should leave m_job_lists_iterator at the current m_*_jobs and m_list_iterator at the current m_job
}

/*! @brief Increments the iterator to reference the next job. If there are no more jobs the iterator will be in the .end() state
 */
JobListIterator& JobListIterator::operator++() 
{
    if (m_job == NULL)
        return *this;
    
    ++m_list_iterator;      // Note. end() refers to past the end, so we increment and then check if we are past the end
    if (m_list_iterator == (*m_job_lists_iterator)->end())
        moveToNextList();
    m_job = *(m_list_iterator);
    return *this;
}

/*! @brief Increments the iterator to reference the next job. If there are no more jobs the iterator will be in the .end() state
 */
JobListIterator& JobListIterator::operator++(int) 
{
    if (m_job == NULL)
        return *this;
    
    ++m_list_iterator;      // Note. end() refers to past the end, so we increment and then check if we are past the end
    if (m_list_iterator == (*m_job_lists_iterator)->end())
        moveToNextList();
    m_job = *(m_list_iterator);
    return *this;
}

/*! @brief Returns true if the two iterators reference the same job
 */
bool JobListIterator::operator==(const JobListIterator& rhs) 
{
    return m_job==rhs.m_job;
}

/*! @brief Returns true when the two iterators reference different jobs
 */
bool JobListIterator::operator!=(const JobListIterator& rhs) 
{
    return m_job!=rhs.m_job;
}

/*! @brief Get the job to which the iterator refers
 */
Job* JobListIterator::operator*()
{
    return m_job;
};

/*! @brief Move to the next non-empty job list in JobList
 */
void JobListIterator::moveToNextList()
{
    for(m_job_lists_iterator++; m_job_lists_iterator!=m_joblist->m_job_lists.end(); ++m_job_lists_iterator)
    {
        if (!(*m_job_lists_iterator)->empty())
        {
            m_list_iterator = (*m_job_lists_iterator)->begin();
            break;
        }
    }
}

