/*! @file BehaviourFSMProvider.h
    @brief Declaration of an abstract behaviour provider class for other behaviours to inherit from
 
    @class BehaviourFSMProvider
    @brief Declaration of an abstract behaviour provider class

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

#ifndef BEHAVIOURFSMPROVIDER_H
#define BEHAVIOURFSMPROVIDER_H

class Behaviour;
#include "Behaviour/BehaviourProvider.h"
class BehaviourState;

class JobList;
class NUSensorsData;
class NUActionatorsData;
class FieldObjects;
class GameInformation;
class TeamInformation;

#include <vector>

class BehaviourFSMProvider : public BehaviourProvider
{
public:
    virtual ~BehaviourFSMProvider();
    
    void process(JobList* jobs, NUSensorsData* data, NUActionatorsData* actions, FieldObjects* fieldobjects, GameInformation* gameinfo, TeamInformation* teaminfo);
protected:
    BehaviourFSMProvider(Behaviour* manager);

    void addState(BehaviourState* state);
    virtual void doBehaviour();

protected:
    BehaviourState* m_state;
    BehaviourState* m_previous_state;
    bool m_state_changed;
    std::vector<BehaviourState*> m_states;
};


#endif

