/*! @file BehaviourFSMState.h
    @brief Declaration of an abstract behaviour state class for other states to inherit from
 
    @class BehaviourFSMState
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

#ifndef BEHAVIOURFSMSTATE_H
#define BEHAVIOURFSMSTATE_H

class NUSensorsData;
class NUActionatorsData;
class JobList;
class FieldObjects;
class GameInformation;
class TeamInformation;

#include "BehaviourState.h"

class BehaviourFSMState : public BehaviourState
{
public:
    virtual ~BehaviourFSMState() {};
    virtual bool stateChanged() {return m_state_changed;};
    virtual bool wasPreviousState(BehaviourState* state) {return state == m_previous_state;};
protected:
    BehaviourFSMState()
    {
        m_state = 0;
        m_previous_state = 0;
        m_state_changed = true;
    };
    virtual BehaviourState* nextState()
    {
        return this;
    };
    void doState();
    virtual void doStateCommons() {};
    virtual BehaviourState* nextStateCommons()
    {  
        return m_state;
    };

protected:
    BehaviourState* m_state;
    BehaviourState* m_previous_state;
    bool m_state_changed;
};


#endif

