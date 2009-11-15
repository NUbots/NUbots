/*! @file Job.h
    @brief Declaration of base Job class.
 
    @class Job
    @brief A base class to encapsulate jobs issued by behaviour.
 
    All jobs need to inherit from this base class. You need only implement a constructor.
    Unfortunately, you need to add the job_id to the job_id_t enum here!

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
    STATE,
    SYSTEM
};

/*! An enum for specific job ids (ie. Stand, Kick etc)
 */
enum job_id_t {
    // Body job ids
    STAND,
    WALK,
    KICK,
    BLOCK,
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
    RESOLUTION,
    FPS,
    SETTINGS,
    AUTO_EXPOSURE,
    AUTO_WHITE_BALANCE,
    AUTO_GAIN,
    BRIGHTNESS,
    CONTRAST,
    SATURATION,
    RED_CHROMA,
    BLUE_CHROMA,
    GAIN,
    EXPOSURE,
    SELECT_CAMERA,
    // Sound job ids
    SUBSTITUTE,
    INITIAL,
    READY,
    SET,
    PLAYING,
    PENALTY,
    FINISH,
    HURT,
    DEAD,
    CRASH,
    LOW_BATTERY,
    HIGH_CURRENT,
    HIGH_TEMPERATURE,
    // System jobs
    SLEEP,
    SHUTDOWN
};

class Job
{
public:
    ~Job();
    
    job_type_t getJobType();
    job_id_t getJobID();
    float getJobTime();
    vector<float>* getPosition();
    vector<float>* getSpeed();
    vector<float>* getCentre();
    vector<float>* getLimits();
    vector<float>* getValues();
    vector<float>* getTarget();

protected:
    job_type_t m_job_type;              //!< The type of job
    job_id_t m_job_id;                  //!< The job's id
    float m_job_time;                   //!< The time the job is to be completed by (seconds)
    vector<float> m_position;           //!< The relative cartesian position at which the job will be completed at 
    vector<float> m_speed;              //!< The speed at which the job will be performed (ie the walk speed)
    vector<float> m_centre;             //!< The job's centre value
    vector<float> m_limits;             //!< The job's min and maximum limits
    vector<float> m_values;             //!< The values used by the job
    vector<float> m_target;             //!< The target of the job (eg. for a kick it is the kick target)
};

#endif

