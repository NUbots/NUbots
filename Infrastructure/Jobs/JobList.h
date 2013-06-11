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


class JobListIterator;      // definition is below JobList

class JobList
{
public:
    friend class JobListIterator;
    typedef JobListIterator iterator;
public:
    JobList();
    ~JobList();
    
    // Add job interface
    void addJob(Job* job);
    void addVisionJob(Job* job);
    void addLocalisationJob(Job* job);
    void addBehaviourJob(Job* job);
    void addMotionJob(Job* job);
    void addCameraJob(Job* job);
    void addSystemJob(Job* job);
    void addOtherJob(Job* job);
    
    // Remove job interface
    std::list<Job*>::iterator removeJob(std::list<Job*>::iterator iter);
    std::list<Job*>::iterator removeVisionJob(std::list<Job*>::iterator iter);
    std::list<Job*>::iterator removeLocalisationJob(std::list<Job*>::iterator iter);
    std::list<Job*>::iterator removeBehaviourJob(std::list<Job*>::iterator iter);
    std::list<Job*>::iterator removeMotionJob(std::list<Job*>::iterator iter);
    void clearMotionJobs();
    std::list<Job*>::iterator removeCameraJob(std::list<Job*>::iterator iter);
    std::list<Job*>::iterator removeSystemJob(std::list<Job*>::iterator iter);
    std::list<Job*>::iterator removeOtherJob(std::list<Job*>::iterator iter);
    
    // Iterators over the jobs
    iterator begin();
    iterator end();
    std::list<Job*>::iterator vision_begin();
    std::list<Job*>::iterator vision_end();
    std::list<Job*>::iterator localisation_begin();
    std::list<Job*>::iterator localisation_end();
    std::list<Job*>::iterator behaviour_begin();
    std::list<Job*>::iterator behaviour_end();
    std::list<Job*>::iterator motion_begin();
    std::list<Job*>::iterator motion_end();
    std::list<Job*>::iterator camera_begin();
    std::list<Job*>::iterator camera_end();
    std::list<Job*>::iterator system_begin();
    std::list<Job*>::iterator system_end();
    std::list<Job*>::iterator other_begin();
    std::list<Job*>::iterator other_end();
    
    void clear();
    bool empty();
    unsigned int size();
    
    void summaryTo(std::ostream& output);
    void csvTo(std::ostream& output);
    
    friend std::ostream& operator<<(std::ostream& output, JobList& joblist);
    friend std::istream& operator>>(std::istream& input, JobList& joblist);
    
private:
    void addJob(Job* job, std::list<Job*>& joblist);
    std::list<Job*>::iterator removeJob(std::list<Job*>& joblist, std::list<Job*>::iterator iter);

private:
    std::list<Job*> m_vision_jobs;               //!< a list of all the current vision jobs
    std::list<Job*> m_localisation_jobs;         //!< a list of all the current localisation jobs
    std::list<Job*> m_behaviour_jobs;            //!< a list of all the behaviour jobs
    std::list<Job*> m_motion_jobs;               //!< a list of all the current motion jobs
    std::list<Job*> m_camera_jobs;               //!< a list of all the current camera jobs
    std::list<Job*> m_system_jobs;               //!< a list of all the current system/os jobs
    std::list<Job*> m_other_jobs;                //!< a list of all other jobs
    std::list<std::list<Job*>*> m_job_lists;          //!< a list of all the lists of jobs
};


/*! @class JobListIterator
    @brief A stl::iterator over a JobList. It iterates over all elements in the list regards this of their type.
 */
class JobListIterator : public std::iterator<std::forward_iterator_tag, Job*>
{
public:
    JobListIterator();
    JobListIterator(JobList* joblist, bool end=false);
    JobListIterator& operator++();
    JobListIterator& operator++(int);
    bool operator==(const JobListIterator& rhs);
    bool operator!=(const JobListIterator& rhs);
    Job* operator*();
private:
    void moveToNextList();
private:
    JobList* m_joblist;
    Job* m_job;                                                 //!< the current job
    std::list<Job*>::iterator m_list_iterator;                       //!< an iterator over the current lists in m_job_lists
    std::list<std::list<Job*>*>::iterator m_job_lists_iterator;           //!< an iterator over the lists in m_job_lists
};

#endif

