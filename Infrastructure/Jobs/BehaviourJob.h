/*! @file BehaviourJob.h
    @brief Declaration of base BehaviourJob class.
 
    @class BehaviourJob
    @brief A base class to encapsulate jobs issued for the behaviour module.
 
    All behaviour jobs should inherit from this base class.
 
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

#ifndef BEHAVIOURJOB_H
#define BEHAVIOURJOB_H

#include "Job.h"

class BehaviourJob : public Job
{
public:
    BehaviourJob(job_id_t jobid) : Job(Job::BEHAVIOUR, jobid){};
    virtual ~BehaviourJob() {};
    
    virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    virtual ostream& operator<< (ostream& output);
    virtual istream& operator>> (istream& input);
};

#endif

