/*! @file MotionKillJob.h
    @brief Declaration of MotionKillJob class.
 
    @class MotionKillJob
    @brief Kills all motion for at least 5 seconds. Each motion module will
           be unkilled when they each receive a new job. A kill is stronger than 
           a freeze; it will put the robot into the safe position and turn off the stiffnesses
 
    @author Jason Kulk
 
  Copyright (c) 2010 Jason Kulk
 
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

#ifndef MOTIONKILLJOB_H
#define MOTIONKILLJOB_H

#include "../MotionJob.h"
#include <vector>
using namespace std;

class MotionKillJob : public MotionJob
{
public:
    MotionKillJob();
    ~MotionKillJob();
    
    virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    friend ostream& operator<<(ostream& output, const MotionKillJob& job);
    friend ostream& operator<<(ostream& output, const MotionKillJob* job);
protected:
    virtual void toStream(ostream& output) const;
};

#endif

