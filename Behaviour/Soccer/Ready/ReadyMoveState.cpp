/*! @file ReadyMoveState.cpp
    @brief Implementation of the initial soccer state

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

#include "ReadyMoveState.h"

#include "Behaviour/GameInformation.h"

#include "Behaviour/Jobs/JobList.h"
#include "Vision/FieldObjects/FieldObjects.h"

#include "Behaviour/Jobs/MotionJobs/HeadTrackJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadPanJob.h"
#include "Behaviour/Jobs/MotionJobs/WalkJob.h"

#include "debug.h"

ReadyMoveState::ReadyMoveState(SoccerFSMState* parent) : SoccerState(parent)
{
}

ReadyMoveState::~ReadyMoveState()
{
}

BehaviourState* ReadyMoveState::nextState()
{
    return this;
}

void ReadyMoveState::doState()
{
    if (m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].isObjectVisible())
        m_jobs->addMotionJob(new HeadTrackJob(m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST]));
    else if (m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].isObjectVisible())
        m_jobs->addMotionJob(new HeadTrackJob(m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST]));
    else if (m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].isObjectVisible())
        m_jobs->addMotionJob(new HeadTrackJob(m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST]));
    else if (m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].isObjectVisible())
        m_jobs->addMotionJob(new HeadTrackJob(m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST]));
    else
        m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation));
    
    vector<float> position = getReadyFieldPositions();
    debug << "Position On Field: " << position[0]<< position[1] <<endl;
    
    vector<float> result = m_field_objects->self.CalculateDifferenceFromFieldState(position);
    debug << "Result: " << result[0]<< result[1] << result [2]<<endl;
    
    m_jobs->addMotionJob(new WalkJob(0.5*result[0], result[1], result[2]/2.0));
}

vector<float> ReadyMoveState::getReadyFieldPositions()
{
	vector<float> position;
	position.reserve(2);
	position.push_back(0.0);
	position.push_back(0.0);
	
	//debug << "Getting GameInformation" << endl;
	//debug << "PlayerNumber \t" << m_game_info->getPlayerNumber() <<endl;
	//debug << "Team \t" << m_game_info->getTeamColour() <<endl;
	//debug << "We have Kickoff \t" << m_game_info->haveKickoff() << endl;
	//debug << "Initial Position \t" << position[0] <<  position[1]<<endl;
	if(m_game_info->getPlayerNumber() == 1)
	{
		position[0] = 280.0;
		position[1] = 0.0;
	}
	else if(m_game_info->getPlayerNumber() == 2)
	{
		if(m_game_info->haveKickoff())
		{
			position[0] = 15.0;
			position[1] = 0.0;
		}
		else
		{
			position[0] = 140.0;
			position[1] = 100.0;
		}
	}
	else if(m_game_info->getPlayerNumber() == 3)
	{
		//debug << "Before player 3 Position \t" << position[0] <<   ","<<position[1]<<endl;
		if(m_game_info->haveKickoff())
		{
			position[0] = 75.0;
			position[1] = -75.0;
		}
		else
		{
			position[0] = 140.0;
			position[1] = -100.0;
		}
		//debug << "Player 3 Position \t" << position[0] <<  ","<<  position[1]<<endl;
	}
	//debug << "Position On Field: " << position[0]<< position[1] <<endl;
	if(m_game_info->getTeamColour() == TEAM_BLUE)
	{
		position[0] = -position[0];
	}
	//debug << "Position On Field: " << position[0]<< position[1] <<endl;
	return position;
}
