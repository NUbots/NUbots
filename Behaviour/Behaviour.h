/*! @file Behaviour.h
    @brief Declaration of the top-level behaviour class
 
    @class Behaviour
    @brief The top-level class for the Behaviour module. 
 
    The Behaviour module has a single BehaviourProvider. The BehaviourProvider can
    be changed online.

    @author Jason Kulk
 
  Copyright (c) 2009,2010 Jason Kulk
 
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

#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

class BehaviourProvider;

class NUSensorsData;
class NUActionatorsData;
class JobList;
class FieldObjects;
class GameInformation;
class TeamInformation;

#include <vector>
#include <string>

class Behaviour
{
public:
    Behaviour();
    ~Behaviour();
    void process(JobList* jobs, NUSensorsData* data, NUActionatorsData* actions, FieldObjects* fieldobjects, GameInformation* gameinfo, TeamInformation* teaminfo);
    
    void setNextBehaviour(std::string name);
    void setNextBehaviour(BehaviourProvider* behaviour);
    
private:
    BehaviourProvider* nameToProvider(std::string name);
    std::string simplifyName(const std::string& input);
private:
    BehaviourProvider* m_behaviour;             //!< the current behaviour provider
    BehaviourProvider* m_next_behaviour;        //!< the next behaviour provider. This will be NULL when we wish to continue to use m_behaviour (ie most of the time).
};


#endif

