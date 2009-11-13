/*! @file BodyJob.h
    @brief Declaration of body job class. 
 
    @class BodyJob
    @brief A class to encapsulate jobs issued by behaviour to control the body of a robot. 
    Typically, these are walk commands.

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

#ifndef BODYJOB_H
#define BODYJOB_H

#include "Behaviour/Job.h"

class BodyJob : public Job
{
public:
    static BodyJob* newStandJob(float time, vector<float> position);
    static BodyJob* newKickJob(float time, vector<float> position, vector<float> kicktarget);
    static BodyJob* newSaveJob(float time, vector<float> position);
private:
    BodyJob(job_id_t jobid, float time, vector<float> position, vector<float> jobtarget);
};

#endif