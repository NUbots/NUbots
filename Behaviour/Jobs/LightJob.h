/*! @file LightJob.h
    @brief Declaration of light job class.
 
    @author Jason Kulk
 
    @class LightJob
    @brief A class to encapsulate jobs issued by behaviour for the robot's lights.
 
    Specify colours using vector<float> (red, green, blue) where red/green/blue range from 0 to 1.0.
    If a non-zero time is specified the colour will be smoothly changed from the current colour.

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

#include "Behaviour/Job.h"

class LightJob : public Job
{
public:
    static LightJob* newLEyeJob(float time, vector<float> colour);
    static LightJob* newREyeJob(float time, vector<float> colour);
    static LightJob* newLEarJob(float time, vector<float> colour);
    static LightJob* newREarJob(float time, vector<float> colour);
    static LightJob* newChestJob(float time, vector<float> colour);
    static LightJob* newLFootJob(float time, vector<float> colour);
    static LightJob* newRFootJob(float time, vector<float> colour);
    
private:
    LightJob(job_id_t jobid, float time, vector<float> colour);
};

#endif

