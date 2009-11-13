/*! @file BodyJob.cpp
    @brief Partial implementation of base job class

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

#include "BodyJob.h"

/*! @brief Create a new stand job at the given time and position
 
    The idea is that you are creating a job that will have the robot 'stand' at the given
    position at the given time. Effectively, if the relative position is non-zero it will
    make the robot walk so that it arrives at the given position at the given time. However, if
    the relative position is zero the robot will stand still.
 
    @param time the time at which the stand job will be completed
    @param position the relative position at which to stand
    
    @return the new BodyJob encapsulating the desired stand job
 */
BodyJob* BodyJob::newStandJob(float time, vector<float> position)
{
    static vector<float> empty_vector;
    BodyJob* job = new BodyJob::BodyJob(STAND, time, position, empty_vector);
    return job;
}

/*! @brief Create a new kick job at the given time and position, to the given target
 
 This creates a job that will have the robot 'kick' the given point
 at the given time in the given direction. If the kick can not be achieved from the current 
 position the robot will walk to a point from which it can perform the kick.
 
 @param time the time at which the stand job will be completed
 @param position the point to be kicked (ie. the position of the ball!)
 @param kicktarget the target for the kick from the point to be kicked (ie. the relative position of the goal FROM the ball)
 
 @return the new BodyJob encapsulating the desired kick job
 */
BodyJob* BodyJob::newKickJob(float time, vector<float> position, vector<float> kicktarget)
{
    BodyJob* job = new BodyJob::BodyJob(KICK, time, position, kicktarget);
    return job;
}

/*! @brief Create a new save job at the given time and position
 
 This creates a job that will stop the ball at the given time and position. The diving motions
 are emergent; when the time gets short enough a dive is the only way to stop a ball at that point.
 Otherwise, the robot will walk toward the point. To position the goalie you should use STAND jobs
 
 @param time the time at which the stand job will be completed
 @param position the relative position at which to stand
 
 @return the new BodyJob encapsulating the desired stand job
 */
BodyJob* BodyJob::newSaveJob(float time, vector<float> position)
{
    static vector<float> empty_vector;
    BodyJob* job = new BodyJob::BodyJob(STAND, time, position, empty_vector);
    return job;
}



BodyJob::BodyJob(job_id_t jobid, float time, vector<float> position, vector<float> jobtarget)
{
    m_job_type = BODY;
    m_job_id = jobid;
    m_job_time = time;
    m_position = position;
    m_target = jobtarget;
}

