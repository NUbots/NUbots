/*! @file SystemJob.h
    @brief Declaration of base SystemJob class.
 
    @class SystemJob
    @brief A base class to encapsulate jobs issued for the system module.
 
    All system jobs should inherit from this base class.
 
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

#ifndef SYSTEMJOB_H
#define SYSTEMJOB_H

#include "Job.h"

class SystemJob : public Job
{
public:
    SystemJob(job_id_t jobid) : Job(Job::SYSTEM, jobid){};
    virtual ~SystemJob() {};
    
    virtual void summaryTo(std::ostream& output);
    virtual void csvTo(std::ostream& output);
    
    virtual std::ostream& operator<< (std::ostream& output);
    virtual std::istream& operator>> (std::istream& input);
};

#endif

