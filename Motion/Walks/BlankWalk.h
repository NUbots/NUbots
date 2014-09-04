/*! @file BlankWalk.h
    @brief Declaration a blank walk class
 
    @class BlankWalk
    @brief A module to provide a placeholder for robots which can not walk (for example a webcam)
 
    @author Jason Kulk
 
  Copyright (c) 2011 Jason Kulk
 
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

#ifndef BLANKWALK_H
#define BLANKWALK_H

#include "Motion/NUWalk.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include <fstream>


class BlankWalk : public NUWalk
{
public:
    BlankWalk(NUSensorsData* data, NUActionatorsData* actions) : NUWalk(data, actions) {};
    ~BlankWalk() {};
protected:
    void doWalk() {};
};

#endif

