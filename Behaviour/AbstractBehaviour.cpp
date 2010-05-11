/*! @file Behaviour.cpp
    @brief Implementation of behaviour class

    @author Jason Kulk
 
 Copyright (c) 2009 Jason Kulk
 
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

#include "AbstractBehaviour.h"
#include "Jobs/JobList.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Vision/FieldObjects/FieldObjects.h"
#include "GameController/GameInformation.h"
#include "TeamInformation.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

AbstractBehaviour::~AbstractBehaviour()
{
}

bool AbstractBehaviour::preProcess(JobList* jobs, NUSensorsData* data, NUActionatorsData* actions, FieldObjects* fieldobjects, GameInformation* gameinfo, TeamInformation* teaminfo)
{
    if (jobs == NULL or data == NULL or actions == NULL or fieldobjects == NULL or gameinfo == NULL or teaminfo == NULL)
        return false;
    else
    {
        m_jobs = jobs;
        m_data = data;
        m_actions = actions;
        m_field_objects = fieldobjects;
        m_game_info = gameinfo;
        m_team_info = teaminfo;
        
        m_previous_time = m_current_time;
        m_current_time = m_data->CurrentTime;
        return true;
    }
}

void AbstractBehaviour::postProcess()
{
}

void AbstractBehaviour::process(JobList* jobs, NUSensorsData* data, NUActionatorsData* actions, FieldObjects* fieldobjects, GameInformation* gameinfo, TeamInformation* teaminfo)
{
    cout << "AbstractBehaviour::process" << endl;
    if (preProcess(jobs, data, actions, fieldobjects, gameinfo, teaminfo))
    {
        if (m_behaviour != NULL)
            m_behaviour->doBehaviour();
        postProcess();
    }
}

