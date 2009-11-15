/*! @file SystemJob.cpp
    @brief Implementation of system job class

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

#include "SystemJob.h"

/*! @brief Put the robot to sleep
    @return the job
 */
SystemJob* SystemJob::newSleepJob()
{
    SystemJob* job = new SystemJob(SLEEP);
    return job;
}

/*! @brief Turn the robot off
    @return the job
 */
SystemJob* SystemJob::newShutdownJob()
{
    SystemJob* job = new SystemJob(SHUTDOWN);
    return job;
}

/*! A private constructor
 */
SystemJob::SystemJob(job_id_t jobid)
{
    m_job_type = SYSTEM;
    m_job_id = jobid;
}


