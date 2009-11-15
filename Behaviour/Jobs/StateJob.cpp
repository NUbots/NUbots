/*! @file StateJob.cpp
    @brief Implementation of state job class

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

#include "StateJob.h"

/*! @brief Create a new initial state job
 @returns the StateJob
 */
StateJob* StateJob::newSubstituteJob()
{
    StateJob* job = new StateJob(SUBSTITUTE);
    return job;
}

/*! @brief Create a new initial state job
    @returns the StateJob
 */
StateJob* StateJob::newInitialJob()
{
    StateJob* job = new StateJob(INITIAL);
    return job;
}

/*! @brief Create a new ready state job
 @returns the StateJob
 */
StateJob* StateJob::newReadyJob()
{
    StateJob* job = new StateJob(READY);
    return job;
}

/*! @brief Create a new set state job
 @returns the StateJob
 */
StateJob* StateJob::newSetJob()
{
    StateJob* job = new StateJob(SET);
    return job;
}

/*! @brief Create a new playing state job
 @returns the StateJob
 */
StateJob* StateJob::newPlayingJob()
{
    StateJob* job = new StateJob(PLAYING);
    return job;
}

/*! @brief Create a new penalty state job
 @returns the StateJob
 */
StateJob* StateJob::newPenaltyJob()
{
    StateJob* job = new StateJob(PENALTY);
    return job;
}

/*! @brief Create a new finish state job
 @returns the StateJob
 */
StateJob* StateJob::newFinishJob()
{
    StateJob* job = new StateJob(FINISH);
    return job;
}

/*! @brief Create a new hurt state job
 @returns the StateJob
 */
StateJob* StateJob::newHurtJob()
{
    StateJob* job = new StateJob(HURT);
    return job;
}

/*! @brief Create a new dead state job
 @returns the StateJob
 */
StateJob* StateJob::newDeadJob()
{
    StateJob* job = new StateJob(DEAD);
    return job;
}

/*! @brief Create a new crash state job
 @returns the StateJob
 */
StateJob* StateJob::newCrashJob()
{
    StateJob* job = new StateJob(CRASH);
    return job;
}

/*! @brief Create a new low battery state job
 @returns the StateJob
 */
StateJob* StateJob::newLowBatteryJob()
{
    StateJob* job = new StateJob(LOW_BATTERY);
    return job;
}

/*! @brief Create a new high current state job
 @returns the StateJob
 */
StateJob* StateJob::newHighCurrentJob()
{
    StateJob* job = new StateJob(HIGH_CURRENT);
    return job;
}

/*! @brief Create a new high temperature state job
 @returns the StateJob
 */
StateJob* StateJob::newHighTemperatureJob()
{
    StateJob* job = new StateJob(HIGH_TEMPERATURE);
    return job;
}

/*! A private constructor for robot state jobs
 */
StateJob::StateJob(job_id_t jobid)
{
    m_job_type = STATE;
    m_job_id = jobid;
}
