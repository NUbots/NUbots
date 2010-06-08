/*! @file BehaviourState.h
    @brief Declaration of an abstract behaviour state class for other states to inherit from
 
    @class BehaviourState
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

#ifndef BEHAVIOURSTATE_H
#define BEHAVIOURSTATE_H

class NUSensorsData;
class NUActionatorsData;
class JobList;
class FieldObjects;
class GameInformation;
class TeamInformation;

class BehaviourProvider;

class BehaviourState
{
public:
    virtual ~BehaviourState();
    BehaviourState* getNextState();
    void process(JobList* jobs, NUSensorsData* data, NUActionatorsData* actions, FieldObjects* fieldobjects, GameInformation* gameinfo, TeamInformation* teaminfo);
protected:
    BehaviourState() {};
    virtual void doState() = 0;
    virtual BehaviourState* nextState() = 0;
protected:
    NUSensorsData* m_data;
    NUActionatorsData* m_actions;
    JobList* m_jobs;
    FieldObjects* m_field_objects;
    GameInformation* m_game_info;
    TeamInformation* m_team_info;
private:
    bool m_processed;           //!< This flag indicates that the Behaviour state's process function has been called
};


#endif

