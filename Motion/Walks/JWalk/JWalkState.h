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


class JWalkState
{
public:
    JWalkState(const std::string& name, const NUData::id_t& leg);
    virtual ~JWalkState() {};
    virtual void doIt() = 0;
    virtual JWalkState* next() = 0;
    
    std::string& getName();
protected:
    std::string m_name;
    NUData::id_t m_leg;
    NUData::id_t m_other_leg;
    
    // Note. I've chosen to go with pointers to pointers so I can do everything in the constructor,
    // and i don't need to keep updating m_state and m_other_state this way.
    JWalkState** m_state;			//!< the current state of this side
    JWalkState** m_stance;			//!< the stance state of this side
    JWalkState** m_push;			//!< the push state of this side
    JWalkState** m_swing;			//!< the swing state of this side
    JWalkState** m_accept;			//!< the accept state of this side
    
    JWalkState** m_other_state;		//!< the current state of the other side
    JWalkState** m_other_stance;	//!< the stance state of the other side
    JWalkState** m_other_push;		//!< the push state of the other side
    JWalkState** m_other_swing;		//!< the swing state of the other side
    JWalkState** m_other_accept;	//!< the accept state of the other side
};

#endif

