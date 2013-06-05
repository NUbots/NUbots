/*! @file NUSoccerStates.h
    @brief NUSoccer actions to execute

    @author Josiah Walker

  Copyright (c) 2013 Josiah Walker

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

#ifndef NUSOCCERSTATES_H
#define NUSOCCERSTATES_H

#include "Behaviour/BehaviourState.h"
#include "Behaviour/Common/HeadBehaviour.h"
#include "NUSoccerProvider.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"
#include "Infrastructure/Jobs/MotionJobs/MotionFreezeJob.h"
#include "Infrastructure/GameInformation/GameInformation.h"

#include "debug.h"




class NUSoccerSubState : public BehaviourState
{
public:
    NUSoccerSubState(NUSoccerProvider* provider){m_provider = provider;}
protected:
    NUSoccerProvider* m_provider;
};

// -----------------------------------------------------------------------------------------------------------------------
// NOTE: the golden rule of this class is DO NOT DO CALCULATIONS.
// Handle all calculations in the Behaviour/Common libraries
// This class is only for logic and commands.

class NUSoccerState : public NUSoccerSubState
{

//all private functions are simple soccer behaviours
private:
    
    void stopMoving(logic,movement,head) { //this is the freeze for penalised/pickup/initial
        movement->stop();
    }
    
    goToStartDefensePositions(logic,movement,head) { //defensive fielding position
        std::vector<float> position = NavigationLogic::getStartDefensePosition();
        movement->goToPoint(position[0],position[1],position[2]);
        head->prioritiseLocalisation();
    }
    
    goToStartOffensePositions(logic,movement,head) { //offensive fielding position
        std::vector<float> position = NavigationLogic::getStartDefensePosition();
        movement->goToPoint(position[0],position[1],position[2]);
        head->prioritiseLocalisation();
    }
    
    watchTheBall(logic,movement,head) {
        head->lookAtBall();
    }
    
    doFieldLocalisation(logic,movement,head) {
        movement->fieldLocalisationTurn();
        head->lookForFieldObjects();
    }
    
    doBallLocalisation(logic,movement,head) {
        movement->ballLocalisationTurn();
        head->lookForBall();
    }
    
    doBallApproachAndKick(logic,movement,head) {
        movement->goToBall();
        head->prioritiseBall();
    }
    
    goToOffensiveSupportPosition(logic,movement,head) {
        std::vector<float> position = NavigationLogic::getBallDefensePosition();
        movement->goToPoint(position[0],position[1],position[2]);
        head->prioritiseBall();
    }
    
    goToDefensiveSupportPosition(logic,movement,head) {
        std::vector<float> position = NavigationLogic::getBallOffensePosition();
        movement->goToPoint(position[0],position[1],position[2]);
        head->prioritiseBall();
    }

public:
    NUSoccerState(NUSoccerProvider* provider) : NUSoccerSubState(provider) {}
    BehaviourState* nextState() {return m_provider->m_state;}
    void doState()
    {   
        BehaviourStateLogic* logic = BehaviourStateLogic::getInstance();
        Navigation* movement = Navigation::getInstance();
        HeadBehaviour* head = HeadBehaviour::getInstance();
        
        //update our decision information
        logic->update();
        
        //do action selection logic:
        if (logic->states[BehaviourStateLogic::GAME_STATE_PENALISED] or
            logic->states[BehaviourStateLogic::GAME_STATE_READY] or
            logic->states[BehaviourStateLogic::IS_PICKED_UP]) {
            
            stopMoving(logic,movement,head);
            
        } else if (logic->states[BehaviourStateLogic::GAME_STATE_POSITIONING] and
                   not logic->states[BehaviourStateLogic::GAME_STATE_KICKOFF]) {
            
            goToStartDefensePositions(logic,movement,head);
            
        } else if (logic->states[BehaviourStateLogic::GAME_STATE_POSITIONING] and
                   logic->states[BehaviourStateLogic::GAME_STATE_KICKOFF]) {
            
            goToStartOffensePositions(logic,movement,head);
            
        } else if (logic->states[BehaviourStateLogic::IS_KICKING]) {
            
            watchTheBall(logic,movement,head);
            
        } else if (logic->states[BehaviourStateLogic::IS_FALLEN_OVER] or
                   logic->states[BehaviourStateLogic::IS_GETTING_UP] or
                   logic->states[BehaviourStateLogic::JUST_GOT_UP] or
                   logic->states[BehaviourStateLogic::JUST_PUT_DOWN] or 
                   logic->states[BehaviourStateLogic::JUST_UNPENALISED]) {
                   
            doFieldLocalisation(logic,movement,head);
            
        } else if (logic->states[BehaviourStateLogic::BALL_IS_LOST] and
                   not logic->states[BehaviourStateLogic::TEAM_SEES_BALL]) {
                   
            doBallLocalisation(logic,movement,head);
            
        } else if ((logic->states[BehaviourStateLogic::IS_APPROACHING_BALL] and
                   not logic->states[BehaviourStateLogic::IS_FURTHEST_FROM_BALL]) or
                   logic->states[BehaviourStateLogic::IS_CLOSEST_TO_BALL]) {
                   
            doBallApproachAndKick(logic,movement,head);
            
        } else if (logic->states[BehaviourStateLogic::IS_SECOND_FROM_BALL] and
                   not logic->states[BehaviourStateLogic::IS_GOALKEEPER]) {
            //XXX: we need to know if it's not the closest to the ball but is the closest to the offensive position....
            goToOffensiveSupportPosition(logic,movement,head);
            
        } else if (logic->states[BehaviourStateLogic::IS_SECOND_FROM_BALL] and
                   not logic->states[BehaviourStateLogic::IS_GOALKEEPER]) {
            //XXX: see above
            goToDefensiveSupportPosition(logic,movement,head);
            
        } /*else if () {
            
        } else if () {
            
        } else if () {
            
        } else if () {
            
        } else if () {
            
        } else if () {
            
        } else if () {
            
        } else if () {
            
        }*/
        
        
        
        
        //update movement and head:
        movement->update();
        head->update();
        
        
    };
};

#endif

