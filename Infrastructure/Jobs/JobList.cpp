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
#include "debugverbosityjobs.h"

/*! @brief JobList constructor
 */
JobList::JobList()
{
    m_job_lists.push_back(&m_vision_jobs);
    m_job_lists.push_back(&m_localisation_jobs);
    m_job_lists.push_back(&m_behaviour_jobs);
    m_job_lists.push_back(&m_motion_jobs);
    m_job_lists.push_back(&m_camera_jobs);
    m_job_lists.push_back(&m_system_jobs);
    m_job_lists.push_back(&m_other_jobs);
}

/*! @brief Job destructor
 */
JobList::~JobList()
{
    debug << "JobList::~JobList()" << std::endl;
    for (JobListIterator it = begin(); it != end(); ++it)
    {
        debug << "deleting something" << std::endl;
        if (*it != NULL)
            delete *it;
    }
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
    else if (jobtype == Job::CAMERA)
        addCameraJob(job);
    else if (jobtype == Job::SYSTEM)
        addSystemJob(job);
    else if (jobtype == Job::OTHER)
        addOtherJob(job);
    else
        debug << "JobList::addJob. Unknown job type. Your job will not be added." << std::endl;
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

/*! @brief Add a camera job to the list
    @param job the job to be added
 */
void JobList::addCameraJob(Job* job)
{
    addJob(job, m_camera_jobs);
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
void JobList::addJob(Job* job, std::list<Job*>& joblist)
{
    joblist.push_back(job);
}

/*! @brief Remove a job from the list based on its an iterator's position
    @param iter the position of the job you want to remove
    @return the new iterator position post job-removal
 */
std::list<Job*>::iterator JobList::removeJob(std::list<Job*>::iterator iter)
{
    Job* job = *iter;
    if (job == NULL)
        return iter;
    Job::job_type_t jobtype = job->getType();
    if (jobtype == Job::MOTION)
        return removeMotionJob(iter);
    else if (jobtype == Job::VISION)
        return removeVisionJob(iter);
    else if (jobtype == Job::LOCALISATION)
        return removeLocalisationJob(iter);
    else if (jobtype == Job::BEHAVIOUR)
        return removeBehaviourJob(iter);
    else if (jobtype == Job::CAMERA)
        return removeCameraJob(iter);
    else if (jobtype == Job::SYSTEM)
        return removeSystemJob(iter);
    else if (jobtype == Job::OTHER)
        return removeOtherJob(iter);
    else
    {
        errorlog << "JobList::removeJob. Unknown job type. Your job was never added to begin with." << std::endl;
        return iter;
    }
}

/*! @brief Remove a vision job from the list
    @param iter the position of the job to be removed
    @return the new iterator position post job-removal
 */
std::list<Job*>::iterator JobList::removeVisionJob(std::list<Job*>::iterator iter)
{
    return removeJob(m_vision_jobs, iter);
}

/*! @brief Remove a localisation job from the list
    @param iter the position of the job to be removed
    @return the new iterator position post job-removal
 */
std::list<Job*>::iterator JobList::removeLocalisationJob(std::list<Job*>::iterator iter)
{
    return removeJob(m_localisation_jobs, iter);
}

/*! @brief Remove a behaviour job from the list
    @param iter the position of the job to be removed
    @return the new iterator position post job-removal
 */
std::list<Job*>::iterator JobList::removeBehaviourJob(std::list<Job*>::iterator iter)
{
    return removeJob(m_behaviour_jobs, iter);
}

/*! @brief Remove a motion job from the list
    @param iter the position of the job to be removed
    @return the new iterator position post job-removal
 */
std::list<Job*>::iterator JobList::removeMotionJob(std::list<Job*>::iterator iter)
{
    return removeJob(m_motion_jobs, iter);
}

/*! @brief Removes all motion jobs from the list
 */
void JobList::clearMotionJobs()
{
    m_motion_jobs.clear();
}

/*! @brief Remove a camera job from the list
    @param iter the position of the job to be removed
    @return the new iterator position post job-removal
 */
std::list<Job*>::iterator JobList::removeCameraJob(std::list<Job*>::iterator iter)
{
    return removeJob(m_camera_jobs, iter);
}

/*! @brief Remove a system job from the list
    @param iter the position of the job to be removed
    @return the new iterator position post job-removal
 */
std::list<Job*>::iterator JobList::removeSystemJob(std::list<Job*>::iterator iter)
{
    return removeJob(m_system_jobs, iter);
}

/*! @brief Remove an other job from the list
    @param iter the position of the job to be removed
    @return the new iterator position post job-removal
 */
std::list<Job*>::iterator JobList::removeOtherJob(std::list<Job*>::iterator iter)
{
    return removeJob(m_other_jobs, iter);
}

/*! @brief Remove a job from the passed in list
    @param joblist the list from which the job will be removed
    @param iter the position in the list of the job to be removed
    @return the new iterator position post job-removal
 */
std::list<Job*>::iterator JobList::removeJob(std::list<Job*>& joblist, std::list<Job*>::iterator iter)
{
    delete *iter;
    return joblist.erase(iter);
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
std::list<Job*>::iterator JobList::vision_begin()
{
    return m_vision_jobs.begin();
}

/*! @brief Returns an iterator at the end of the vision jobs.
 */
std::list<Job*>::iterator JobList::vision_end()
{
    return m_vision_jobs.end();
}

/*! @brief Returns an iterator at the beginning of the localisation jobs.
 */
std::list<Job*>::iterator JobList::localisation_begin()
{
    return m_localisation_jobs.begin();
}

/*! @brief Returns an iterator at the end of the localisation jobs.
 */
std::list<Job*>::iterator JobList::localisation_end()
{
    return m_localisation_jobs.end();
}

/*! @brief Returns an iterator at the beginning of the behaviour jobs.
 */
std::list<Job*>::iterator JobList::behaviour_begin()
{
    return m_behaviour_jobs.begin();
}

/*! @brief Returns an iterator at the end of the behaviour jobs.
 */
std::list<Job*>::iterator JobList::behaviour_end()
{
    return m_behaviour_jobs.end();
}

/*! @brief Returns an iterator at the beginning of the motion jobs.
 */
std::list<Job*>::iterator JobList::motion_begin()
{
    return m_motion_jobs.begin();
}

/*! @brief Returns an iterator at the end of the motion jobs.
 */
std::list<Job*>::iterator JobList::motion_end()
{
    return m_motion_jobs.end();
}

/*! @brief Returns an iterator at the beginning of the camera jobs.
 */
std::list<Job*>::iterator JobList::camera_begin()
{
    return m_camera_jobs.begin();
}

/*! @brief Returns an iterator at the end of the camera jobs.
 */
std::list<Job*>::iterator JobList::camera_end()
{
    return m_camera_jobs.end();
}

/*! @brief Returns an iterator at the beginning of the system jobs.
 */
std::list<Job*>::iterator JobList::system_begin()
{
    return m_system_jobs.begin();
}

/*! @brief Returns an iterator at the end of the system jobs.
 */
std::list<Job*>::iterator JobList::system_end()
{
    return m_system_jobs.end();
}

/*! @brief Returns an iterator at the beginning of the other jobs.
 */
std::list<Job*>::iterator JobList::other_begin()
{
    return m_other_jobs.begin();
}

/*! @brief Returns an iterator at the end of the other jobs.
 */
std::list<Job*>::iterator JobList::other_end()
{
    return m_other_jobs.end();
}

/*! @brief Clears the contents of the job list
 */
void JobList::clear()
{
    std::list<std::list<Job*>*>::iterator it;
    for (it = m_job_lists.begin(); it != m_job_lists.end(); it++)
        (*it)->clear();
}

/*! @brief Returns true if the JobList is empty
 */
bool JobList::empty()
{
    std::list<std::list<Job*>*>::iterator it;
    for (it = m_job_lists.begin(); it != m_job_lists.end(); it++)
    {
        if (not (*it)->empty())
            return false;
    }
    return true;
}

/*! @brief Returns the size of the job list
    @return Returns the size of the job list 
 */
unsigned int JobList::size()
{
    unsigned int size = 0;
    std::list<std::list<Job*>*>::iterator it;
    for (it = m_job_lists.begin(); it != m_job_lists.end(); it++)
        size += (*it)->size();
    return size;
}

void JobList::summaryTo(std::ostream& output)
{
    output << "JobList " << size() << " -----------------------------------" << std::endl;
    static JobList::iterator it;     // the iterator over all of the jobs
    if (size() != 0)
    {   //<! @todo TODO. Investigate why we get a segment fault when the JobList size is zero!
        for (it = begin(); it != end(); ++it)
            (*it)->summaryTo(output);
    }
    output << "-------------------------------------------" << std::endl;
}

void JobList::csvTo(std::ostream& output)
{
    static JobList::iterator it;     // the iterator over all of the jobs
    for (it = begin(); it != end(); ++it)
        (*it)->csvTo(output);
}

std::ostream& operator<<(std::ostream& output, JobList& joblist)
{
#if DEBUG_JOBS_VERBOSITY > 4
    debug << "ostream << JobList. " << joblist.size() << " jobs." << std::endl;
#endif
    output << joblist.size() << " ";
    
    static JobList::iterator it;     // the iterator over all of the jobs
    for (it = joblist.begin(); it != joblist.end(); ++it)
        output << *it;
    return output;
}

std::istream& operator>>(std::istream& input, JobList& joblist)
{
    #if DEBUG_JOBS_VERBOSITY > 4
        debug << "istream >> JobList" << std::endl;
    #endif
    char buffer[8];
    unsigned int numnewjobs = 0;
    input >> numnewjobs;
    input.read(buffer, sizeof(char));       // skip over the white space
    #if DEBUG_JOBS_VERBOSITY > 4
        debug << "istream >> JobList. Adding " << numnewjobs << std::endl;
    #endif
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
#if DEBUG_JOBS_VERBOSITY > 5
    debug << "JobListIterator::JobListIterator. Contents of JobList:" << std::endl;
    std::list<std::list<Job*>*>::iterator it;
    std::list<Job*>::iterator sit;
    for (it = m_joblist->m_job_lists.begin(); it != m_joblist->m_job_lists.end(); ++it)
    {
        for (sit = (*it)->begin(); sit!=(*it)->end(); ++sit)
            debug << (*sit) << " ";
    }
    debug << std::endl;
#endif
    
    if (m_joblist->empty())
    {
        m_list_iterator = (*m_joblist->m_job_lists.begin())->begin();
        m_job = *m_list_iterator;
    }
    else
    {
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
            std::list<std::list<Job*>*>::reverse_iterator reverseit;
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

