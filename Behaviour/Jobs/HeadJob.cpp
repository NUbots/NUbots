/*! @file HeadJob.cpp
    @brief Implementation of head job class

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

#include "HeadJob.h"


/*! @brief Create a new point tracking job
 
    Directly, control the position of the head using this command. For example, track the ball using this function
    by calculating the head angles required to put the ball at the right place in the image.
 
    @param time the time in seconds at which to reach the given point (use this to control the speed; set to 0 for as fast as possible)
    @param point the point (head_yaw, head_pitch, [head_roll]) in radians to track
 */
HeadJob* HeadJob::newTrackJob(float time, vector<float> point)
{
    HeadJob* job = new HeadJob::HeadJob(PAN, time, point);
    return job;
}

/*! @brief Create a new nod job
 
    A job to move the head up and down periodically.
 
    @param period the period of the nod in seconds
    @param centre the nod's centre (yaw, pitch, roll) in radians
    @param limits the nod's (pitch) limits in radians
 */
HeadJob* HeadJob::newNodJob(float period, vector<float> centre, vector<float> limits)
{
    HeadJob* job = new HeadJob::HeadJob(NOD, period, centre, limits);
    return job;
}

/*! @brief Create a new pan job
 
    A job to pan the head from side to side periodically.
 
    @param period the period of the pan in seconds
    @param centre the pan's centre (yaw, pitch, roll) in radians
    @param limits the nod's (yaw) limits in radians
 */
HeadJob* HeadJob::newPanJob(float period, vector<float> centre, vector<float> limits)
{
    HeadJob* job = new HeadJob::HeadJob(PAN, period, centre, limits);
    return job;
}


/*! A private constructor for HeadJobs.
 */
HeadJob::HeadJob(job_id_t jobid, float time, vector<float> centre, vector<float> limits)
{
    m_job_type = HEAD;
    m_job_id = jobid;
    m_job_time = time;
    m_centre = centre;
    m_limits = limits;
}

/*! A private contructor for HeadJobs with a position
 */
HeadJob::HeadJob(job_id_t jobid, float time, vector<float> position)
{
    m_job_type = HEAD;
    m_job_id = jobid;
    m_job_time = time;
    m_position = position;
}

