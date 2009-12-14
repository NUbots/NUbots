/*! @file REyeLedJob.h
    @brief Declaration of base REyeLedJob class.
 
    @class REyeLedJob
    @brief A class to encapsulate jobs issued to the left eye module.
 
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

#ifndef REYELEDJOB_H
#define REYELEDJOB_H

#include "../LightJob.h"

class REyeLedJob : public LightJob
{
public:
    REyeLedJob(double time, const vector<float>& colour) : LightJob(Job::LIGHT_R_EYE, time, colour) {};
    ~REyeLedJob() {};
};

#endif

