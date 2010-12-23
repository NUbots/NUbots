/*! @file JWalkState.h
    @brief Declaration of a jwalk state
 
    @class JWalkState
    @brief An abstract jwalk state
 
    @author Jason Kulk
 
  Copyright (c) 2010, 2011 Jason Kulk
 
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

#ifndef JWALKSTATE_H
#define JWALKSTATE_H

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "JWalk.h"

#include <string>
using namespace std;

class JWalkState
{
public:
    JWalkState(const string& name, const NUData::id_t& leg) 
    {
        Name = name;
        Leg = leg;
    };
    virtual ~JWalkState() {};
    virtual void doIt() = 0;
    virtual JWalkState* next() = 0;
    string Name;
    NUData::id_t Leg;
};

#endif

