/*! @file PlayingState.h
    @brief Declaration of the playing soccer state
 
    @class PlayingState
    @brief The playing soccer state
 
    The top-level playing state machine. There are only 4 states at this level:
        - ChaseState where the robot will chase and kick the ball
        - PositioningState where the robot will position in a offensive or defensive position
        - BallIsLost where the robot will search for the ball
        - ImLost where the robot will attempt to localise
 
    We are in the ball is lost state if the ball is lost and we are a field player, no exceptions.
    We are in the chasing state if we are the closest to the ball, no exceptions.
    We are in the lost state if we are lost.
    We are in the positioning state if we are not closest to the ball.

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

#ifndef PLAYING_FSM_STATE_H
#define PLAYING_FSM_STATE_H

class SoccerProvider;
#include "../SoccerFSMState.h"

class ChaseState;
class PositioningState;
class BallIsLostState;
class ImLostState;

#include <vector>
using namespace std;

class PlayingState : public SoccerFSMState
{
public:
    PlayingState(SoccerProvider* provider);
    virtual ~PlayingState();
protected:
    BehaviourFSMState* nextState();
    virtual void doStateCommons();
    virtual BehaviourFSMState* nextStateCommons();
    ChaseState* m_chase_state;
    PositioningState* m_positioning_state;
    BallIsLostState* m_ball_is_lost_state;
    ImLostState* m_im_lost_state;
private:
    vector<float> m_led_on;
    vector<float> m_led_off;
    vector<float> m_led_red;
    vector<float> m_led_green;
    vector<float> m_led_yellow;
};


#endif

