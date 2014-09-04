/*! @file VisionJob.h
    @brief Declaration of base VisionJob class.
 
    @class VisionJob
    @brief A base class to encapsulate jobs issued for the vision module.
 
    All vision jobs should inherit from this base class.
 
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

#ifndef VISIONJOB_H
#define VISIONJOB_H

#include "Job.h"

class VisionJob : public Job
{
public:
    VisionJob(job_id_t jobid) : Job(Job::VISION, jobid) {};
    virtual ~VisionJob() {};
    
    virtual void summaryTo(std::ostream& output) = 0;
    virtual void csvTo(std::ostream& output) = 0;
    
    friend std::ostream& operator<< (std::ostream& output, const VisionJob& job) 
    {   job.toStream(output); 
        return output;
    };
    friend std::ostream& operator<< (std::ostream& output, const VisionJob* job)
    {
        if (job != NULL) 
            job->toStream(output);
        else
            output << "NULL";
        return output;
    };
protected:
    virtual void toStream(std::ostream& output) const {(void)(output); /* To stop compiler warnings.*/}
};

#endif

