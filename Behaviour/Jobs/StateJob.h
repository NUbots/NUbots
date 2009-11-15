/*! @file StateJob.h
    @brief Declaration of state job class.
    @author Jason Kulk
 
    @class StateJob
    @brief A class to encapsulate jobs issued by behaviour for changing the robot's state.

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

#ifndef STATEJOB_H
#define STATEJOB_H

#include "Behaviour/Jobs/Jobs.h"

class StateJob : public Job
{
public:
    static StateJob* newSubstituteJob();
    static StateJob* newInitialJob();
    static StateJob* newReadyJob();
    static StateJob* newSetJob();
    static StateJob* newPlayingJob();
    static StateJob* newPenaltyJob();
    static StateJob* newFinishJob();
    static StateJob* newHurtJob();
    static StateJob* newDeadJob();
    static StateJob* newCrashJob();
    static StateJob* newLowBatteryJob();
    static StateJob* newHighCurrentJob();
    static StateJob* newHighTemperatureJob();
    
private:
    StateJob(job_id_t jobid);
};

#endif

