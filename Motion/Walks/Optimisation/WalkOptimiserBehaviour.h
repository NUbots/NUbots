/*! @file WalkOptimiserBehaviour.h
    @brief Declaration of WalkOptimiserBehaviour class
 
    @class WalkOptimiserBehaviour
    @brief A module to provide control the robot while walk optimisation is running
 
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

#ifndef WALKOPTIMISERBEHAVIOUR_H
#define WALKOPTIMISERBEHAVIOUR_H

#include "Behaviour/Jobs.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "WalkParameters.h"

#include <fstream>
using namespace std;

class WalkOptimiserBehaviour
{
public:
    WalkOptimiserBehaviour();
    ~WalkOptimiserBehaviour();
    
    void process(NUSensorsData* data, NUActionatorsData* actions, JobList& joblist);
protected:
private:
    bool isFallen();
public:
protected:
private:
    NUSensorsData* m_data;
    NUActionatorsData* m_actions;
    WalkParameters* m_walk_parameters;
};

#endif

