/*! @file Job.h
    @brief Declaration of base Job class.
 
    @class Job
    @brief A base class to encapsulate jobs issued by behaviour.
 
    All jobs need to inherit from this base class. 

    So I need to add jobs to the job list. All of the jobs have different data members
    I would like to keep it as simple as possible, but also avoid each end having to know
    any sort of order. I could have a structure for each type.
 
    I have tried to have many Job inherit from a base job class. This did not work too well,
    because the data for each job is so different.
 
 Option 1. I could redo it with inheritance but assume that I am allowed to cast the job pointer
           to be of the right type. I am not sure that doing inheritance based on the job type is so greate
           because even within the same job type the data is pretty different. So if I was going to use inheritance
           I would need to have a class for each of the 50 odd jobs. That is a LOT of classes. 
 
           Some of them are pretty similar: All of the camera jobs are the same. All of the light jobs are the same. All of the sound jobs are the same.
           All of the system jobs are the same. So only VISION, LOCALISATION, BEHAVIOUR and MOTION have jobs with vastly different input data.
 
           Alright just cast the motherfuckers so that you can get the right data!
 
           MotionJob()
 
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
        MOTION_STAND,
        MOTION_WALK,
        MOTION_KICK,
        MOTION_BLOCK,
        MOTION_SAVE,
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
    Job();
    ~Job();
    
    job_type_t getType();
    job_id_t getID();
    double getTime();
    
    /*virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    virtual ostream& operator<< (ostream& output);
    virtual istream& operator>> (istream& input);*/

protected:
    // Properties that *every* job has
    job_type_t m_job_type;              //!< The type of job
    job_id_t m_job_id;                  //!< The job's id
    double m_job_time;                   //!< The time the job is to be completed (milliseconds)
};

#endif

