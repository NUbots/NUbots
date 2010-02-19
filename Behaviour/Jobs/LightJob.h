/*! @file LightJob.h
    @brief Declaration of base LightJob class.
 
    @class LightJob
    @brief A base class to encapsulate jobs issued for the lights module.
 
    All light jobs should inherit from this base class.
 
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

#ifndef LIGHTJOB_H
#define LIGHTJOB_H

#include "Job.h"

class LightJob : public Job
{
public:
    LightJob(job_id_t jobid, double time, const vector<float>& colour) : Job(Job::LIGHT, jobid){m_job_time = time; m_colour = colour;};
    virtual ~LightJob() {m_colour.clear();};
    
    void setColour(double time, const vector<float>& colour) {m_job_time = time; m_colour = colour;};
    void getColour(double& time, vector<float>& colour) {time = m_job_time; colour = m_colour;};
    
    virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    virtual ostream& operator<< (ostream& output);
    virtual istream& operator>> (istream& input);
protected:
    vector<float> m_colour;
    
};

#endif

