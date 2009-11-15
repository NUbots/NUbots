/*! @file BodyJob.cpp
    @brief Implementation of body job class

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
 
    @param time the time in seconds at which the stand job will be completed
    @param position the relative position at which to stand: (x,y,theta) in centimetres and radians
    
    @return the new BodyJob encapsulating the desired stand job
 */
BodyJob* BodyJob::newStandJob(float time, vector<float> position)
{
    static vector<float> empty_vector;
    BodyJob* job = new BodyJob::BodyJob(STAND, time, position, empty_vector);
    return job;
}

/*! @brief Create a new walk job at the given speed
 
 This creates a fairly low-level walk job. This function can accurately control motion's
 underlying omnidirectional walk.
 
 @param speed the walk speed in x, y and theta (cm/s, cm/s, rad/s)
 
 @return the new BodyJob encapsulating the desired walk job
 */
BodyJob* BodyJob::newWalkJob(vector<float> speed)
{
    BodyJob* job = new BodyJob::BodyJob(WALK, speed);
    return job;
}


/*! @brief Create a new kick job at the given time and position, to the given target
 
 This creates a job that will have the robot 'kick' the given point
 at the given time in the given direction. If the kick can not be achieved from the current 
 position the robot will walk to a point from which it can perform the kick.
 
 @param time the time in seconds at which the kick job will be completed
 @param position the point (x,y,theta) to be kicked (ie. the position of the ball!) in centimetres and radians
 @param kicktarget the target (distance, angle) for the kick from the point to be kicked (ie. the relative position of the goal FROM the ball) in centimetres and radians
 
 @return the new BodyJob encapsulating the desired kick job
 */
BodyJob* BodyJob::newKickJob(float time, vector<float> position, vector<float> kicktarget)
{
    BodyJob* job = new BodyJob::BodyJob(KICK, time, position, kicktarget);
    return job;
}

/*! @brief Create a new block job at the given time and position
 
 This creates a job that will stop the ball at the given time and position without using the hands.
 Consequently, this function is safe for fieldplayers and goalies outside their boxes. The diving motions
 are emergent; when the time gets short enough a dive is the only way to stop a ball at that point.
 Otherwise, the robot will walk toward the point. To position the goalie you should use STAND jobs
 
 @param time the time in seconds at which the 'ball' will pass throught the given point
 @param position the relative position at which to block: (x,y, theta) in centimetres and radians
 
 @return the new BodyJob encapsulating the desired block job
 */
BodyJob* BodyJob::newBlockJob(float time, vector<float> position)
{
    BodyJob* job = new BodyJob::BodyJob(BLOCK, time, position);
    return job;
}

/*! @brief Create a new save job at the given time and position
 
 This creates a job that will stop the ball at the given time and position. The diving motions
 are emergent; when the time gets short enough a dive is the only way to stop a ball at that point.
 Otherwise, the robot will walk toward the point. To position the goalie you should use STAND jobs
 
 @param time the time in seconds at which the 'ball' will pass throught the given point
 @param position the relative position at which to block: (x,y, theta) in centimetres and radians
 
 @return the new BodyJob encapsulating the desired save job
 */
BodyJob* BodyJob::newSaveJob(float time, vector<float> position)
{
    BodyJob* job = new BodyJob::BodyJob(SAVE, time, position);
    return job;
}


/*! A private constructor for body jobs that only have a speed.
 
    It is private because I provide static functions to create specific types of body jobs.
    I could probably have the specific jobs inherit from the BodyJob but why?
 */
BodyJob::BodyJob(job_id_t jobid, vector<float> speed)
{
    m_job_type = BODY;
    m_job_id = jobid;
    m_speed = speed;
}

/*! A private constructor for body jobs that only have a final position and time
 
 It is private because I provide static functions to create specific types of body jobs.
 I could probably have the specific jobs inherit from the BodyJob but why?
 */
BodyJob::BodyJob(job_id_t jobid, float time, vector<float> position)
{
    m_job_type = BODY;
    m_job_id = jobid;
    m_job_time = time;
    m_position = position;
}

/*! A private constructor for body jobs that have a final position and time, and an action target
 
 It is private because I provide static functions to create specific types of body jobs.
 I could probably have the specific jobs inherit from the BodyJob but why?
 */
BodyJob::BodyJob(job_id_t jobid, float time, vector<float> position, vector<float> jobtarget)
{
    m_job_type = BODY;
    m_job_id = jobid;
    m_job_time = time;
    m_position = position;
    m_target = jobtarget;
}
