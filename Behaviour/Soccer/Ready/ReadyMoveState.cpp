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

#include "Behaviour/BehaviourPotentials.h"

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
    // I want to do smart pan based on where we are in the field
    // if fabs(heading) < pi/2
    //      if (distance from yellow goal > 4.2)
    //          pan to see the half way line and the yellow goals
    //      else
    //          pan to see the yellow goals
    
    StationaryObject& yellow_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST];
    StationaryObject& yellow_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST];
    StationaryObject& blue_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST];
    StationaryObject& blue_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST];
    
    if (yellow_left.isObjectVisible() and yellow_right.isObjectVisible())
    {
        float bearing = (yellow_left.ScreenXTheta() + yellow_right.ScreenXTheta())/2;
        float elevation = (yellow_left.ScreenYTheta() + yellow_right.ScreenYTheta())/2;
        m_jobs->addMotionJob(new HeadTrackJob(elevation, bearing));
    }
    else if (blue_left.isObjectVisible() and blue_right.isObjectVisible())
    {
        float bearing = (blue_left.ScreenXTheta() + blue_right.ScreenXTheta())/2;
        float elevation = (blue_left.ScreenYTheta() + blue_right.ScreenYTheta())/2;
        m_jobs->addMotionJob(new HeadTrackJob(elevation, bearing));
    }
    else if (yellow_left.TimeSinceLastSeen() > 250 and yellow_right.TimeSinceLastSeen() > 250 and blue_left.TimeSinceLastSeen() > 250 and blue_right.TimeSinceLastSeen() > 250)
        m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation));
    
    
    
    vector<float> position = getReadyFieldPositions();
    vector<float> speed = BehaviourPotentials::goToFieldState(m_field_objects->self, position, 5, 60, 60);
    vector<float> result = BehaviourPotentials::sensorAvoidObjects(speed, m_data, 30, 100);
    m_jobs->addMotionJob(new WalkJob(result[0], result[1], result[2]));
}

vector<float> ReadyMoveState::getReadyFieldPositions()
{
	vector<float> position(3,0);
    
	
	//debug << "Getting GameInformation" << endl;
	//debug << "PlayerNumber \t" << m_game_info->getPlayerNumber() <<endl;
	//debug << "Team \t" << m_game_info->getTeamColour() <<endl;
	//debug << "We have Kickoff \t" << m_game_info->haveKickoff() << endl;
	//debug << "Initial Position \t" << position[0] <<  position[1]<<endl;
	if(m_game_info->getPlayerNumber() == 1)
	{
		position[0] = 290.0;
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
			position[1] = 0.0;
		}
	}
	else if(m_game_info->getPlayerNumber() == 3)
	{
		//debug << "Before player 3 Position \t" << position[0] <<   ","<<position[1]<<endl;
		if(m_game_info->haveKickoff())
		{
			position[0] = 60.0;
			position[1] = 100.0;
		}
		else
		{
			position[0] = 140.0;
			position[1] = 90.0;
		}
		//debug << "Player 3 Position \t" << position[0] <<  ","<<  position[1]<<endl;
	}
	//debug << "Position On Field: " << position[0]<< position[1] <<endl;
	if(m_game_info->getTeamColour() == TEAM_BLUE)
	{
		position[0] = -position[0];
        position[1] = -position[1];
        position[2] = 0;
	}
    else
    {
        position[2] = 3.141;
    }

	//debug << "Position On Field: " << position[0]<< position[1] <<endl;
	return position;
}
