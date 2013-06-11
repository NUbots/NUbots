/*! @file CameraJob.h
    @brief Declaration of base CameraJob class.
 
    @class CameraJob
    @brief A base class to encapsulate jobs issued for the camera module.
 
    All camera jobs should inherit from this base class.
 
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

#ifndef CAMERAJOB_H
#define CAMERAJOB_H

#include "Job.h"

class CameraJob : public Job
{
public:
    CameraJob(job_id_t jobid) : Job(Job::CAMERA, jobid) {};
    virtual ~CameraJob() {};
    
    virtual void summaryTo(std::ostream& output) = 0;
    virtual void csvTo(std::ostream& output) = 0;
    
    friend std::ostream& operator<< (std::ostream& output, const CameraJob& job) 
    {   job.toStream(output); 
        return output;
    };
    friend std::ostream& operator<< (std::ostream& output, const CameraJob* job)
    {
        if (job != NULL) 
            job->toStream(output);
        else
            output << "NULL";
        return output;
    };
protected:
    virtual void toStream(std::ostream& output) const {};
};

#endif

