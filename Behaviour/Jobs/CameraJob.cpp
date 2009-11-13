/*! @file Job.cpp
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

#include "Job.h"

/*! @brief Job destructor
 */
Job::~Job()
{
    // I think everything will cleaning delete itself.
}

/*! @brief Get the job's type
    @returns the job's type
 */
job_type_t Job::getJobType()
{
    return m_job_type;
}

/*! @brief Get the job's id
    @returns the job's id
 */
job_id_t Job::getJobID()
{
    return m_job_id;
}

/*! @brief Get the time at which the job is to be completed
    @returns the job time
 */
float Job::getJobTime()
{
    return m_job_time;
}

/*! @brief Get the position at which the final task in the job will be executed
    @returns the position the job will be executed at
 */
vector<float>* Job::getPosition()
{
    return &m_position;
}

/*! @brief Get the values used by the job
    @returns the values used by the job
 */
vector<float>* Job::getValues()
{
    return &m_values;
}

/*! @brief Get the target of the job
    @return the target of the job
 */
vector<float>* Job::getTarget()
{
    return &m_target;
}

BodyJob* BodyJob::newStandJob(float time, vector<float> position)
{
    static vector<float> empty_vector;
    BodyJob* temp = new BodyJob::BodyJob(STAND, time, position, empty_vector);
    return temp;
}

BodyJob::BodyJob(job_id_t jobid, float time, vector<float> position, vector<float> jobtarget)
{
    m_job_type = BODY;
    m_job_id = jobid;
    m_job_time = time;
    m_position = position;
    m_target = jobtarget;
}

HeadJob* HeadJob::newNodJob(float period)
{
    static vector<float> empty_vector;
    HeadJob* temp = new HeadJob::HeadJob(NOD, period, empty_vector);
    return temp;
}

HeadJob::HeadJob(job_id_t jobid, float time, vector<float> position)
{
    m_job_type = HEAD;
    m_job_id = jobid;
    m_job_time = time;
    m_position = position;
}

