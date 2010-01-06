/*! @file NBWalk.h
    @brief Declaration of Northern Bites's walk engine 
 
    @class NBWalk
    @brief A module to provide locomotion using Northern Bites 2009 walk.
 
    This module is uses Johannes Strom's omni-directional walk engine detailed in the paper:
    An Implementation of an Omnidirectional Walk Engine on a Nao Robot, RoboCup 2009.
 
    As the Northern Bites are kind enough to share their code, most of the code for the implementation
    of this walk engine was written by them. The header of each file will indicate the author of that
    file, I will mark files I needed to modify.
 
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

#ifndef NBWALK_H
#define NBWALK_H

#include "Motion/NUWalk.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"

#include <fstream>
using namespace std;

class NBWalk : public NUWalk
{
public:
    NBWalk();
    ~NBWalk();
protected:
    void doWalk();
private:
    
public:
protected:
private:
    
    // Pattern generation debugging
    ofstream m_pattern_debug;
};

#endif

