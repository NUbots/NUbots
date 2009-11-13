/*! @file Job.h
    @brief Declaration of base Job class.
 
    @class Job
    @brief A base class to encapsulate jobs issued by behaviour.
 
    All jobs need to inherit from this base class. You need only implement a constructor.

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

#ifndef JOB_H
#define JOB_H

#include <vector>
using namespace std;

/*! An enum for job types (ie. Body, Head, etc)
 */
enum job_type_t {
    BODY,
    HEAD,
    LIGHT,
    CAMERA,
    SOUND,
    SYSTEM
};

/*! An enum for specific job ids (ie. Stand, Kick etc)
 */
enum job_id_t {
    // Body job ids
    STAND,
    KICK,
    SAVE,
    // Head job ids
    TRACK,
    NOD,
    PAN,
    // Light job ids
    L_EYE,
    R_EYE,
    L_EAR,
    R_EAR,
    CHEST,
    L_FOOT,
    R_FOOT,
    // Camera job ids
    EXPOSURE,
    GAIN,
    SELECT_CAMERA,
    // Sound job ids
    INITIAL,
    READY,
    SET,
    PLAYING,
    PENALTY,
    FINISH,
    // System jobs
    SLEEP,
    OFF
};

class Job
{
public:
    ~Job();
    
    job_type_t getJobType();
    job_id_t getJobID();
    float getJobTime();
    vector<float>* getPosition();
    vector<float>* getValues();
    vector<float>* getTarget();

protected:
    job_type_t m_job_type;              //!< The type of job
    job_id_t m_job_id;                  //!< The job's id
    float m_job_time;                   //!< The time the job is to be completed by
    vector<float> m_position;           //!< The relative position at which the job will be completed at
    vector<float> m_values;             //!< The values used by the job
    vector<float> m_target;             //!< The target of the job (eg. for a kick it is the kick target)
};


class BodyJob : public Job
{
public:
    static BodyJob* newStandJob(float time, vector<float> position);
    static BodyJob* newKickJob(float time, vector<float> position, vector<float> kicktarget);
    static BodyJob* newSaveJob(float time, vector<float> position);
private:
    BodyJob(job_id_t jobid, float time, vector<float> position, vector<float> jobtarget);
};

class HeadJob : public Job
{
public:
    static HeadJob* newTrackJob(float time, vector<float> position);
    static HeadJob* newNodJob(float period);
    static HeadJob* newPanJob(float period);
    
private:
    HeadJob(job_id_t jobid, float time, vector<float> position);
};

#endif

