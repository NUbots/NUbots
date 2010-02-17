/*! @file MotionJob.h
    @brief Declaration of base MotionJob class.
 
    @class MotionJob
    @brief A base class to encapsulate jobs issued for the motion module.
 
    All motion jobs should inherit from this base class.
 
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

#ifndef MOTIONJOB_H
#define MOTIONJOB_H

#include "Job.h"

class MotionJob : public Job
{
public:
    MotionJob(job_id_t jobid) : Job(Job::MOTION, jobid){};
    virtual ~MotionJob() {};
    
    virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    friend ostream& operator<< (ostream& output, const MotionJob& job);
    friend istream& operator>> (istream& input, MotionJob& job);
};

#endif

