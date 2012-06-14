/*! @file PlayingState.h
    @brief Implementation of the ready soccer state

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

#include "PlayingState.h"
#include "Striker/ChaseState.h"
#include "Support/PositioningState.h"
#include "BallIsLost/BallIsLostState.h"
#include "GoalKeeper/GoalKeeperState.h"
#include "ImLost/ImLostState.h"

#include "../SoccerProvider.h"
#include "Tools/Math/General.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"

PlayingState::PlayingState(SoccerProvider* provider) : SoccerFSMState(provider)
{
    m_chase_state = new ChaseState(this);
    m_positioning_state = new PositioningState(this);
    m_ball_is_lost_state = new BallIsLostState(this);
    m_im_lost_state = new ImLostState(this);
    m_goalkeeper_state = new GoalKeeperState(this);
    
    m_state = m_chase_state;
    
    m_led_on = vector<float>(3,1.0f);
    m_led_off = vector<float>(3,0.0f);
    m_led_red = m_led_off;
    m_led_red[0] = 1;
    m_led_green = m_led_off;
    m_led_green[1] = 1;
    m_led_yellow = m_led_off;
    m_led_yellow[0] = 1;
    m_led_yellow[1] = 1;
}

PlayingState::~PlayingState()
{
    delete m_chase_state;
    delete m_positioning_state;
    delete m_ball_is_lost_state;
    delete m_im_lost_state;
}

void PlayingState::doStateCommons()
{
    if (m_provider->stateChanged())
    {   // play a sound when we enter the playing state, turn the kick off light off
        m_actions->add(NUActionatorsData::Sound, m_actions->CurrentTime, NUSounds::PLAYING);
        m_actions->add(NUActionatorsData::RFootLed, m_actions->CurrentTime, m_led_off);
    }
    // In playing the chest led should be green
    m_actions->add(NUActionatorsData::ChestLed, m_actions->CurrentTime, m_led_green);
    
    // set the right eye leds to indicate which state we are in
    if (m_state == m_chase_state)
        m_actions->add(NUActionatorsData::REyeLed, m_actions->CurrentTime, m_led_red);
    else if (m_state == m_positioning_state)
        m_actions->add(NUActionatorsData::REyeLed, m_actions->CurrentTime, m_led_green);
    else
        m_actions->add(NUActionatorsData::REyeLed, m_actions->CurrentTime, m_led_off);
}

BehaviourFSMState* PlayingState::nextStateCommons()
{   // do state transitions in playing state machine
    if (false and m_team_info->getPlayerNumber() != 1) { //striker state transitions
        if (m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].lost()) {
            Blackboard->lookForBall = true;
            return m_ball_is_lost_state;
        } else if (m_team_info->amIClosestToBall()) {
            return m_chase_state;
        } else if (m_field_objects->self.lost()) {
            Blackboard->lookForBall = true;
            return m_im_lost_state;
        } else {
            Blackboard->lookForBall = true;
            return m_positioning_state;
            }
            
    } else { //goalkeeper state transitions
        m_team_info->setPlayerNumber(1);
        //calculate distance to my own goal position
        float goal_diff_x;
        float goal_diff_y;
        float distance_from_centre_to_goal = fabs(m_field_objects->stationaryFieldObjects[FieldObjects::FO_CORNER_YELLOW_PEN_LEFT].X());
        float heading = mathGeneral::normaliseAngle(3.14-m_field_objects->self.Heading());
        
        if (Blackboard->GameInfo->getTeamColour() == GameInformation::BlueTeam) {
            distance_from_centre_to_goal *= -1.f;
            heading = mathGeneral::normaliseAngle(0.f-m_field_objects->self.Heading());
        }
        
        goal_diff_x = distance_from_centre_to_goal - m_field_objects->self.wmX(); //X value
        goal_diff_y = 0.f - m_field_objects->self.wmY(); //difference from centre
        float distsqr = goal_diff_x * goal_diff_x + goal_diff_y * goal_diff_y;
        
        if (m_field_objects->self.lost())
            return m_im_lost_state;
        //else if (m_team_info->amIClosestToBall())
        //    return m_chase_state;
        else if (distsqr > 10.f*10.f or fabs(heading) > 0.15) //distance to where I should be is too large
            return m_positioning_state;
        //else //if (m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].lost())
        //    return m_ball_is_lost_state;
        else //goalie save state
            return m_goalkeeper_state;
    }
}

BehaviourFSMState* PlayingState::nextState()
{   // the playing state machine can never modify the game state machine
    return this;
}




