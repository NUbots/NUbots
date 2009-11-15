/*! @file HeadJob.h
    @brief Declaration of HeadJob class.
 
    @class HeadJob
    @brief A class to encapsulate jobs issued by behaviour for the head.
 
    There are three types of head jobs
        - Track; use this to effectively move the head into a desired position
        - Nod; use this to move the head up and down while spinning
        - Pan; use this to move the head side to side while searching.

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

#ifndef HEADJOB_H
#define HEADJOB_H

#include "Behaviour/Job.h"

class HeadJob : public Job
{
public:
    static HeadJob* newTrackJob(float time, vector<float> point);
    static HeadJob* newNodJob(float period, vector<float> centre, vector<float> limits);
    static HeadJob* newPanJob(float period, vector<float> centre, vector<float> limits);
    
private:
    HeadJob(job_id_t jobid, float time, vector<float> centre, vector<float> limits);
    HeadJob(job_id_t jobid, float time, vector<float> point);
};

#endif

