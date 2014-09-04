/*! @file BehaviourStateLogic.h
    @brief Logic for head behaviours.
    Main methods involve porting of a vector of values such as: times since objects last seen, object locations, object head movement costs.
    The indexing for each of these lists is identical. The method getObject(int index) gets the object corresponding to the values in the
    slot [index] in each list.

    @author Josiah Walker

 Copyright (c) 2013

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


#ifndef BEHAVIOUR_STATE_LOGIC_H
#define BEHAVIOUR_STATE_LOGIC_H


#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/FieldObjects/Object.h"
#include "NUPlatform/NUCamera/NUCameraData.h"

#include "Infrastructure/NUSensorsData/NUSensorsData.h"


#include "Infrastructure/GameInformation/GameInformation.h"


#include "Tools/Math/General.h"


#include <vector>


class BehaviourStateLogic{

private:
        //timer variables, since nothing in this robot system tracks time
        double timeFallen;
        double timeGetup;
        //double timeGameResumed = 0.0;
        double timeGameStarted;
        double timePutDown;
        double timeUnPenalised;
        int m_GoalKeeper;

public:
    BehaviourStateLogic() {
        timeFallen = -50000.;
        timeGetup = -50000.;
        //timeGameResumed = 0.0;
        timeGameStarted = 0.0;
        timePutDown = -20000.0;
        timeUnPenalised = -20000.0;
        m_GoalKeeper = 1;
        states[IS_CLOSEST_TO_BALL] = false;
        states[IS_SECOND_FROM_BALL] = false;
        states[IS_FURTHEST_FROM_BALL] = false;
        states[IS_FALLEN_OVER] = false;
        states[IS_GETTING_UP] = false;
        states[JUST_GOT_UP] = false;
        states[IS_PICKED_UP] = false;
        states[BALL_IS_SEEN] = false;
        states[TEAM_SEES_BALL] = false;
        states[BALL_IS_LOST] = false;
        states[GAME_STATE_PENALISED] = false;
        
        states[GAME_STATE_INITIAL] = false;
        states[GAME_STATE_SET] = false;
        states[GAME_STATE_READY] = false;
        states[GAME_STATE_KICKOFF] = false;
        states[GAME_STATE_END] = false;
        states[IS_KICKING] = false;
        states[IS_APPROACHING_BALL] = false;
        states[IS_IN_POSITION] = false;
        states[IS_GOAL_KEEPER] = false;
        states[GAME_STATE_PLAYING] = false;
        states[JUST_PUT_DOWN] = false;
        states[JUST_UNPENALISED] = false;
        states[GAME_STATE_KICKING_OFF] = false;
    }

    static BehaviourStateLogic* getInstance();
    
    bool states[30];
    
    enum StateVariableID{
        IS_CLOSEST_TO_BALL = 0,
        IS_SECOND_FROM_BALL = 1,
        IS_FURTHEST_FROM_BALL = 2,
        IS_FALLEN_OVER = 3,
        IS_GETTING_UP = 4,
        JUST_GOT_UP = 5,
        IS_PICKED_UP = 6,
        BALL_IS_SEEN = 7,
        TEAM_SEES_BALL = 8,
        BALL_IS_LOST = 9,
        GAME_STATE_PENALISED = 10,
        GAME_STATE_SET = 11,
        GAME_STATE_READY = 12,
        GAME_STATE_KICKOFF = 13,
        GAME_STATE_END = 14,
        IS_KICKING = 15,
        IS_APPROACHING_BALL = 16,
        IS_IN_POSITION = 17,
        IS_GOAL_KEEPER = 18,
        GAME_STATE_PLAYING = 19,
        JUST_PUT_DOWN = 20,
        JUST_UNPENALISED = 21,
        GAME_STATE_KICKING_OFF = 22,
        GAME_STATE_INITIAL = 23
    };
    
    double timeSinceFallen() {
        return (Blackboard->Sensors->GetTimestamp()-timeFallen);
    }
    
    double timeSinceGetup() {
        return (Blackboard->Sensors->GetTimestamp()-timeGetup);
    }
    
    double timeSinceGameStarted() {
        return (Blackboard->Sensors->GetTimestamp()-timeGameStarted);
    }
    
    double timeSincePutDown() {
        return (Blackboard->Sensors->GetTimestamp()-timePutDown);
    }
    
    double timeSinceUnPenalised() {
        return (Blackboard->Sensors->GetTimestamp()-timeUnPenalised);
    }
    
    //fill in the distance to ball bools
    void checkDistanceToBall();
    
    //fill in the ball visibility/islost bools
    void checkVisibilityOfBall();
    
    //fill in the gamestate bools
    void checkGameState();
    
    //fill in the personal movement goals/position bools
    void checkMyMovement();
    
    //fill in the fallen/picked up bools
    void checkFallen();
    
    //update everything.
    void update();
    
};

#endif
