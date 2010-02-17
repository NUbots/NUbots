/*! @file Job.h
    @brief Declaration of base Job class.
 
    @class Job
    @brief A base class to encapsulate jobs issued by behaviour.
 
    All jobs need to inherit from this base class. 
 
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
        LIGHT,
        CAMERA,
        SOUND,
        SYSTEM,
        OTHER
    };
    /*! @brief An enum for specific job ids (ie. Stand, Kick etc)
     */
    enum job_id_t {
        // Vision job ids
        VISION_LOAD_LUT,
        // Localisation job ids
        LOCALISATION_RESET,
        // Behaviour job ids
        BEHAVIOUR_CHANGE_STRATEGY,
        // Motion job ids
        MOTION_WALK_TO_POINT,
        MOTION_WALK,
        MOTION_KICK,
        MOTION_BLOCK,
        MOTION_SAVE,
        MOTION_HEAD,
        MOTION_TRACK,
        MOTION_NOD,
        MOTION_PAN,
        // Light job ids
        LIGHT_L_EYE,
        LIGHT_R_EYE,
        LIGHT_L_EAR,
        LIGHT_R_EAR,
        LIGHT_CHEST,
        LIGHT_L_FOOT,
        LIGHT_R_FOOT,
        // Camera job ids
        CAMERA_RESOLUTION,
        CAMERA_FPS,
        CAMERA_AUTO_EXPOSURE,
        CAMERA_AUTO_WHITE_BALANCE,
        CAMERA_AUTO_GAIN,
        CAMERA_BRIGHTNESS,
        CAMERA_CONTRAST,
        CAMERA_SATURATION,
        CAMERA_RED_CHROMA,
        CAMERA_BLUE_CHROMA,
        CAMERA_GAIN,
        CAMERA_EXPOSURE,
        CAMERA_SELECT_CAMERA,
        // Sound job ids
        SOUND_SUBSTITUTE,
        SOUND_INITIAL,
        SOUND_READY,
        SOUND_SET,
        SOUND_PLAYING,
        SOUND_PENALTY,
        SOUND_FINISH,
        SOUND_HURT,
        SOUND_DEAD,
        SOUND_CRASH,
        SOUND_LOW_BATTERY,
        SOUND_HIGH_CURRENT,
        SOUND_HIGH_TEMPERATURE,
        // System jobs
        SYSTEM_SLEEP,
        SYSTEM_SHUTDOWN
    };
    
public:
    Job(job_type_t jobtype, job_id_t jobid);
    virtual ~Job();
    
    job_type_t getType();
    job_id_t getID();
    double getTime();
    long double getTimeStamp();
    
    virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    virtual ostream& operator<< (ostream& output);
    virtual istream& operator>> (istream& input);

protected:
    // Properties that *every* job has
    const job_type_t m_job_type;              //!< The type of job (use this to decide which module to send the job to)
    const job_id_t m_job_id;                  //!< The job's id (use this to decide which specialised Job class to cast a Job to)
    double m_job_time;                        //!< The time the job is to be completed (milliseconds)
    long double m_timestamp;                  //!< the unix timestamp the job was generated (milliseconds since epoch)
};

#endif

