/*! @file BodyJob.h
    @brief Declaration of body job class. 
 
    @class BodyJob
    @brief A class to encapsulate jobs issued by behaviour to control the body of a robot. 
    These are walk-action macros.
 
    There are five types of body jobs:
        - Stand; request motion to make the robot stand at a given position
        - Walk; request motion to move at given speed
        - Kick; request motion kick a given point toward a given target
        - Block; request motion to block a given point *without* using your hands
        - Save; request motion to block a given point potentially using your hands
 
    Which function should I use?
        - If you need to precisely control the robot's path, for example, you are working your way through
        obstacles, then use a WalkJob.
        - If you want to kick a ball always use a KickJob
        - If you want to block the ball use the SaveJob if you are a goalie inside your box
        - If you want to block the ball use the BlockJob if you are outside your penaly box, or a fieldplayer
        - If you just want to tell the robot to stand at a position, for example, in ready state use a StandJob.
          This function will do some path planning itself to get to the point at *exactly* the given time

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
    static BodyJob* newWalkJob(vector<float> speed);
    static BodyJob* newKickJob(float time, vector<float> position, vector<float> kicktarget);
    static BodyJob* newBlockJob(float time, vector<float> position);
    static BodyJob* newSaveJob(float time, vector<float> position);
private:
    BodyJob(job_id_t jobid, vector<float> speed);
    BodyJob(job_id_t jobid, float time, vector<float> position);
    BodyJob(job_id_t jobid, float time, vector<float> position, vector<float> jobtarget);
};

#endif