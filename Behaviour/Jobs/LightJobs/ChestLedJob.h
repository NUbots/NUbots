/*! @file ChestLedJob.h
    @brief Declaration of base ChestLedJob class.
 
    @class ChestLedJob
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

#ifndef CHESTLEDJOB_H
#define CHESTLEDJOB_H

#include "../LightJob.h"

class ChestLedJob : public LightJob
{
public:
    ChestLedJob(double time, const vector<float>& colour) : LightJob(Job::LIGHT_CHEST, time, colour) {};
    ~ChestLedJob() {};
};

#endif

