/*! @file SoccerState.h
    @brief Declaration of an abstract behaviour state class for other states to inherit from
 
    @class SoccerState
    @brief Declaration of an abstract behaviour state class for other states to inherit from

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

#ifndef SOCCER_STATE_H
#define SOCCER_STATE_H

class SoccerProvider;
class SoccerFSMState;
#include "Behaviour/BehaviourState.h"

class SoccerState : public BehaviourState
{
public:
    virtual ~SoccerState() {};
protected:
    SoccerState(SoccerProvider* provider) {m_provider = provider; m_parent = 0;};
    SoccerState(SoccerFSMState* parent) {m_parent = parent; m_provider = 0;};
    SoccerProvider* m_provider;
    SoccerFSMState* m_parent;
};


#endif

