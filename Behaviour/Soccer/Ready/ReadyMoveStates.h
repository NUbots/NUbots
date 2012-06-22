/*! @file ReadyMoveStates.h
    @brief Declaration of the ball is lost states

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

#ifndef READY_MOVE_STATES_H
#define READY_MOVE_STATES_H

#include "../SoccerState.h"
#include "ReadyMoveState.h"

#include "Behaviour/BehaviourPotentials.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "Infrastructure/GameInformation/GameInformation.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"

#include "debug.h"
#include "debugverbositybehaviour.h"
using namespace std;

class ReadyMoveSubState : public SoccerState
{
public:
    ReadyMoveSubState(ReadyMoveState* parent) : SoccerState(parent) {};
    virtual ~ReadyMoveSubState() {};
};

// ----------------------------------------------------------------------------------------------------------------------- ReadyMoveWalk
/*! @class ReadyMoveWalk
    ?
 */
class ReadyMoveWalk : public ReadyMoveSubState
{
public:
    ReadyMoveWalk(ReadyMoveState* parent) : ReadyMoveSubState(parent) 
    {
        m_time_in_state = 0;
        m_previous_time = 0;
    }
    ~ReadyMoveWalk() {};
protected:
    BehaviourState* nextState()
    {   // do state transitions in the ready move state
        return this;
    }
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "ReadyMoveWalk" << endl;
        #endif
        // keep track of the time in this state
        if (m_parent->stateChanged())
            m_time_in_state = 0;
        else
            m_time_in_state += m_data->CurrentTime - m_previous_time;
        m_previous_time = m_data->CurrentTime;
        
//        // handle the head movements: For now we look at the yellow or blue goal if we can see them, otherwise pan
//        StationaryObject& yellow_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST];
//        StationaryObject& yellow_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST];
//        StationaryObject& blue_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST];
//        StationaryObject& blue_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST];
        
//        if (yellow_left.isObjectVisible() and yellow_right.isObjectVisible())
//        {
//            float bearing = (yellow_left.ScreenXTheta() + yellow_right.ScreenXTheta())/2;
//            float elevation = (yellow_left.ScreenYTheta() + yellow_right.ScreenYTheta())/2;
//            m_jobs->addMotionJob(new HeadTrackJob(elevation, bearing));
//        }
//        else if (blue_left.isObjectVisible() and blue_right.isObjectVisible())
//        {
//            float bearing = (blue_left.ScreenXTheta() + blue_right.ScreenXTheta())/2;
//            float elevation = (blue_left.ScreenYTheta() + blue_right.ScreenYTheta())/2;
//            m_jobs->addMotionJob(new HeadTrackJob(elevation, bearing));
//        }
//        else if (yellow_left.TimeSinceLastSeen() > 1000 and yellow_right.TimeSinceLastSeen() > 1000 and blue_left.TimeSinceLastSeen() > 1000 and blue_right.TimeSinceLastSeen() > 1000)
            m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation));
        
        vector<float> position = getReadyFieldPositions();
        vector<float> speed = BehaviourPotentials::goToFieldState(m_field_objects->self, position, 0, 50, 100);
        
        vector<float> result;
        if (m_team_info->getPlayerNumber() != 1)
            result = BehaviourPotentials::sensorAvoidObjects(speed, m_data, 50, 100);
        else
            result = BehaviourPotentials::sensorAvoidObjects(speed, m_data);
        m_jobs->addMotionJob(new WalkJob(result[0], result[1], result[2]));
    }
private:
    vector<float> getReadyFieldPositions()
    {
        vector<float> position(3,0);
        if(m_team_info->getPlayerNumber() == 1)
        {
            position[0] = 290.0;
            position[1] = 0.0;
        }
        else if(m_team_info->getPlayerNumber() == 2)
        {
            if(m_game_info->haveKickoff())
            {
                position[0] = 25.0;
                position[1] = 0.0;
            }
            else
            {
                position[0] = 100.0;
                position[1] = 0.0;
            }
        }
        else if(m_team_info->getPlayerNumber() == 3)
        {
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
        }
        else if(m_team_info->getPlayerNumber() == 4)
        {
            if(m_game_info->haveKickoff())
            {
                position[0] = 60.0;
                position[1] = -100.0;
            }
            else
            {
                position[0] = 140.0;
                position[1] = -90.0;
            }
        }
        else if(m_team_info->getPlayerNumber() == 5)
        {
            if(m_game_info->haveKickoff())
            {
                position[0] = 90.0;
                position[1] = 0.0;
            }
            else
            {
                position[0] = 150.0;
                position[1] = 0.0;
            }
        }
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
        return position;
    }
    
    float m_time_in_state;
    double m_previous_time;
};

#endif

