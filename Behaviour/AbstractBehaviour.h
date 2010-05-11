/*! @file AbstractBehaviour.h
    @brief Declaration of an abstract behaviour class for other behaviours to inherit from
 
    @class AbstractBehaviour
    @brief Declaration of an abstract behaviour class

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

#ifndef ABSTRACTBEHAVIOUR_H
#define ABSTRACTBEHAVIOUR_H

class JobList;
class NUSensorsData;
class NUActionatorsData;
class FieldObjects;
class GameInformation;
class TeamInformation;

class AbstractBehaviour
{
public:
    virtual ~AbstractBehaviour();
    
    void process(JobList* jobs, NUSensorsData* data, NUActionatorsData* actions, FieldObjects* fieldobjects, GameInformation* gameinfo, TeamInformation* teaminfo);
protected:
    AbstractBehaviour() {m_behaviour = this;};
    bool preProcess(JobList* jobs, NUSensorsData* data, NUActionatorsData* actions, FieldObjects* fieldobjects, GameInformation* gameinfo, TeamInformation* teaminfo);
    virtual void doBehaviour() = 0;
    virtual void postProcess();

protected:
    double m_current_time;
    double m_previous_time;
    
    JobList* m_jobs;
    NUSensorsData* m_data;
    NUActionatorsData* m_actions;
    FieldObjects* m_field_objects;
    GameInformation* m_game_info;
    TeamInformation* m_team_info;
    
    AbstractBehaviour* m_behaviour;
};


#endif

