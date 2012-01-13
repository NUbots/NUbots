/*! @file DarwinWalk.h
    @brief Declaration of Northern Bites's walk engine 
 
    @class DarwinWalk
    @brief A module to provide locomotion using Darwin Default walk engine.
 
 
    @author Aaron Wong
 
  	Copyright (c) 2011 Aaron Wong, Jason Kulk
 
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

#ifndef DARWINWALK_H
#define DARWINWALK_H

#include "Motion/NUWalk.h"

class NUSensorsData;
class NUActionatorsData;

//From Darwin Library:
#include <Walking.h>	//Darwin Controller

#include <vector>
#include <fstream>
using namespace std;

class DarwinJointMapping;

class DarwinWalk : public NUWalk
{
public:
    DarwinWalk(NUSensorsData* data, NUActionatorsData* actions);
    ~DarwinWalk();
    
	void doWalk();
	void updateActionatorsData();
	void updateWalkEngineSensorData();
private:
	Robot::Walking* DarwinWalkEngine;
	void SetDarwinSensor(int,float);
        float GetDarwinSensor(int walk_id);
        DarwinJointMapping* m_joint_mapping;
};

#endif

