/*! @file MotionJob.cpp
    @brief Implementation of base MotionJob class.
 
    @class MotionJob
    @brief A base class to encapsulate jobs issued for the motion module.
 
    All motion jobs should inherit from this base class.
 
    @author Jason Kulk
 
  Copyright (c) 2009, 2010 Jason Kulk
 
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


#include "MotionJob.h"
#include "debug.h"


/*! @relates MotionJob
    @brief Stream insertion operator for MotionJob.
 
    This operator calls the protected virtual member toStream(output). As toStream is virtual
    the correct toStream function will be called for all types of jobs.

    @param output the stream to put the job in
    @param job the job to put in the stream
 */
ostream& operator<< (ostream& output, const MotionJob& job)
{
    job.toStream(output);
    return output;
}


