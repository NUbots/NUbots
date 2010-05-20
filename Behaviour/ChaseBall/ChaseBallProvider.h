/*! @file ChaseBallProvider.h
    @brief Declaration of simple chase ball behaviour for testing and demonstration purposes 
 
    @class ChaseBallProvider
    @brief A simple chase ball behaviour for testing and demonstration purposes 

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

#ifndef CHASEBALLBEHAVIOUR_H
#define CHASEBALLBEHAVIOUR_H

#include "Behaviour/BehaviourFSMProvider.h"

class ChaseState;
class SearchState;
class PositionState;
class PausedState;

#include <vector>
#include <string>

class ChaseBallProvider : public BehaviourFSMProvider
{
public:
    ChaseBallProvider(Behaviour* manager, bool pauseable = true);
    ~ChaseBallProvider();
protected:
    BehaviourState* nextStateCommons();
private:
    bool m_pauseable;
    
    friend class ChaseState;
    BehaviourState* m_chase_state;
    friend class SearchState;
    BehaviourState* m_search_state;
    friend class PositionState;
    BehaviourState* m_position_state;
    friend class PausedState;
    BehaviourState* m_paused_state;
};


#endif

