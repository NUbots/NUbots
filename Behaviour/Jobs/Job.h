/*! @file Job.h
    @brief Declaration of base Job class.
 
    @class Job
    @brief A base class to encapsulate jobs issued by behaviour.
 
    All jobs need to inherit from this base class. 
 
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

#ifndef JOB_H
#define JOB_H

#include <vector>
#include <iostream>
using namespace std;

class Job
{
public:
    /*! @brief An enum for job types (ie. Body, Head, etc)
     */
    enum job_type_t {
        VISION,
        LOCALISATION,
        BEHAVIOUR,
        MOTION,
        CAMERA,
        SYSTEM,
        OTHER,
        TYPE_UNDEFINED
    };
    /*! @brief An enum for specific job ids (ie. Stand, Kick etc)
     */
    enum job_id_t {
        // Vision job ids
        VISION_LOAD_LUT,
        VISION_SAVE_IMAGES,
        // Localisation job ids
        LOCALISATION_RESET,
        // Behaviour job ids
        BEHAVIOUR_CHANGE_STRATEGY,
        // Motion job ids
        MOTION_WALK_TO_POINT,
        MOTION_WALK,
        MOTION_WALK_PARAMETERS,
        MOTION_KICK,
        MOTION_BLOCK,
        MOTION_SAVE,
        MOTION_SCRIPT,
        MOTION_HEAD,
        MOTION_NOD,
        MOTION_PAN,
        // Camera job ids
        CAMERA_CHANGE_SETTINGS,
        CAMERA_SELECT_CAMERA,
        // System jobs
        SYSTEM_SLEEP,
        SYSTEM_SHUTDOWN,
        // Undefiend
        ID_UNDEFINED
    };
    
public:
    Job(job_type_t jobtype, job_id_t jobid);
    virtual ~Job();
    
    job_type_t getType();
    job_id_t getID();
    double getTime();
    
    virtual void summaryTo(ostream& output) = 0;
    virtual void csvTo(ostream& output) = 0;
    
    friend ostream& operator<<(ostream& output, const Job& job);
    friend ostream& operator<<(ostream& output, const Job* job);
    friend istream& operator>>(istream& input, Job** job);
protected:
    virtual void toStream(ostream& output) const;

protected:
    // Properties that *every* job has
    const job_type_t m_job_type;              //!< The type of job (use this to decide which module to send the job to)
    const job_id_t m_job_id;                  //!< The job's id (use this to decide which specialised Job class to cast a Job to)
    double m_job_time;                        //!< The time the job is to be completed (milliseconds)
};

#endif

