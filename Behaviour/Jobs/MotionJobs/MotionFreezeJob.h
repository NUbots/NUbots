/*! @file MotionFreezeJob.h
    @brief Declaration of MotionFreezeJob class.
 
    @class MotionFreezeJob
    @brief Freezes all motion for at least 5 seconds. Each motion module will
           be unfrozen when they each receive a new job.
 
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

#ifndef MOTIONFREEZEJOB_H
#define MOTIONFREEZEJOB_H

#include "../MotionJob.h"
#include <vector>
using namespace std;

class MotionFreezeJob : public MotionJob
{
public:
    MotionFreezeJob();
    ~MotionFreezeJob();
    
    virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    friend ostream& operator<<(ostream& output, const MotionFreezeJob& job);
    friend ostream& operator<<(ostream& output, const MotionFreezeJob* job);
protected:
    virtual void toStream(ostream& output) const;
};

#endif

