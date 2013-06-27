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
#include "Behaviour/Common/BehaviourStateLogic.h"
#include "Behaviour/Common/Navigation.h"
#include "Behaviour/Common/NavigationLogic.h"
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
#include "Infrastructure/Jobs/VisionJobs/SaveImagesJob.h"

#include "debug.h"




class NUSoccerSubState : public BehaviourState
{
public:
    NUSoccerSubState(NUSoccerProvider* provider){m_provider = provider; saving_images = false;}
protected:
    NUSoccerProvider* m_provider;
    bool saving_images;
};

// -----------------------------------------------------------------------------------------------------------------------
// NOTE: the golden rule of this class is DO NOT DO CALCULATIONS.
// Handle all calculations in the Behaviour/Common libraries
// This class is only for logic and commands.

class NUSoccerState : public NUSoccerSubState
{

//all private functions are simple soccer behaviours
private:
    
    
    void stopMoving(BehaviourStateLogic* logic, Navigation* movement,HeadBehaviour* head) { //this is the freeze for penalised/pickup/initial
        movement->stop();
        head->prioritiseLocalisation();
    }
    
    void goToStartDefensePositions(BehaviourStateLogic* logic, Navigation* movement,HeadBehaviour* head) { //defensive fielding position
        std::vector<float> position = NavigationLogic::getStartDefensePosition();
        
        movement->goToPoint(position);
        head->prioritiseLocalisation();
    }
    
    void goToStartOffensePositions(BehaviourStateLogic* logic, Navigation* movement,HeadBehaviour* head) { //offensive fielding position
        std::vector<float> position = NavigationLogic::getStartOffensePosition();
        movement->goToPoint(position);
        head->prioritiseLocalisation();
    }
    
    void watchTheBall(BehaviourStateLogic* logic, Navigation* movement,HeadBehaviour* head) {
        head->lookAtBall();
    }
    
    void doFieldLocalisation(BehaviourStateLogic* logic, Navigation* movement,HeadBehaviour* head) {
        movement->goToPoint(NavigationLogic::fieldLocalisationPosition());
        head->lookForFieldObjects();
    }
    
    void doBallLocalisation(BehaviourStateLogic* logic, Navigation* movement,HeadBehaviour* head) {
        //movement->goToPoint(NavigationLogic::ballLocalisationPosition());
        movement->goToPoint(0.,0.,0.28);
        head->lookForBall();
    }
    
    void doBallApproachAndKick(BehaviourStateLogic* logic, Navigation* movement,HeadBehaviour* head) {

        movement->goToBall2();
        //head->prioritiseBall();
        head->prioritiseBall();
        movement->kick();        
    }
    
    void goToOffensiveSupportPosition(BehaviourStateLogic* logic, Navigation* movement,HeadBehaviour* head) {
        std::vector<float> position = NavigationLogic::getBallDefensePosition();
        movement->goToPoint(position[0],position[1],position[2]);
        head->prioritiseBall();
    }
    
    void goToDefensiveSupportPosition(BehaviourStateLogic* logic, Navigation* movement,HeadBehaviour* head) {
        std::vector<float> position = NavigationLogic::getBallOffensePosition();
        movement->goToPoint(position[0],position[1],position[2]);
        head->prioritiseBall();
    }

public:
    NUSoccerState(NUSoccerProvider* provider) : NUSoccerSubState(provider) {}
    BehaviourState* nextState() {return m_provider->m_state;}
    void doState()
    {   
        //std::cout << "Beginning Behaviour" << std::endl;
        BehaviourStateLogic* logic = BehaviourStateLogic::getInstance();
        Navigation* movement = Navigation::getInstance();
        HeadBehaviour* head = HeadBehaviour::getInstance();
        
        //update our decision information
        //std::cout << "about to update logic" << std::endl;
        logic->update();
        
        //printout for states
        //XXX: send to NUbugger

        /*
        std::cout <<"IS_CLOSEST_TO_BALL" << logic->states[0] << std::endl <<
                    "IS_SECOND_FROM_BALL" << logic->states[1] << std::endl <<
                    "IS_FURTHEST_FROM_BALL" << logic->states[2] << std::endl <<
                    "IS_FALLEN_OVER" << logic->states[3] << std::endl <<
                    "IS_GETTING_UP" << logic->states[4] << std::endl <<
                    "JUST_GOT_UP" << logic->states[5] << std::endl <<
                    "IS_PICKED_UP" << logic->states[6] << std::endl <<
                    "BALL_IS_SEEN" << logic->states[7] << std::endl <<
                    "TEAM_SEES_BALL" << logic->states[8] << std::endl <<
                    "BALL_IS_LOST" << logic->states[9] << std::endl <<
                    "GAME_STATE_INITIAL" << logic->states[23] << std::endl <<
                    "GAME_STATE_PENALISED" << logic->states[10] << std::endl <<
                    "GAME_STATE_SET" << logic->states[11] << std::endl <<
                    "GAME_STATE_READY" << logic->states[12] << std::endl <<
                    "GAME_STATE_KICKOFF" << logic->states[13] << std::endl <<
                    "GAME_STATE_END" << logic->states[14] << std::endl <<
                    "GAME_STATE_PLAYING" << logic->states[19] << std::endl <<
                    "IS_KICKING" << logic->states[15] << std::endl <<
                    "IS_APPROACHING_BALL" << logic->states[16] << std::endl <<
                    "IS_IN_POSITION" << logic->states[17] << std::endl <<
                    "IS_GOAL_KEEPER" << logic->states[18] << std::endl <<
                    "JUST_PUT_DOWN" << logic->states[20] << std::endl <<
                    "JUST_UNPENALISED" << logic->states[21] << std::endl <<
                    "GAME_STATE_KICKING_OFF" << logic->states[22] << std::endl;
        */
        
        //do action selection logic:
        if (logic->states[BehaviourStateLogic::GAME_STATE_PENALISED] or
            logic->states[BehaviourStateLogic::GAME_STATE_INITIAL] or
            logic->states[BehaviourStateLogic::GAME_STATE_SET] or
            logic->states[BehaviourStateLogic::GAME_STATE_END] or
            logic->states[BehaviourStateLogic::IS_FALLEN_OVER] or
            logic->states[BehaviourStateLogic::IS_PICKED_UP]) {
            
            //XXX: send to NUbugger

            //std::cout << "Stopping Movement" << std::endl;
            
            stopMoving( logic, movement, head);
            
        } else if (logic->states[BehaviourStateLogic::GAME_STATE_READY] and
                   not logic->states[BehaviourStateLogic::GAME_STATE_KICKOFF]) {
            
            //XXX: send to NUbugger

            //std::cout << "Going To Start Position (Non Kickoff)" << std::endl;
            
            goToStartDefensePositions( logic, movement, head);
            
        } else if (logic->states[BehaviourStateLogic::GAME_STATE_READY] and
                   logic->states[BehaviourStateLogic::GAME_STATE_KICKOFF]) {
            
            //XXX: send to NUbugger

            //std::cout << "Going To Start Position (Kickoff)" << std::endl;
            
            goToStartOffensePositions(logic, movement, head);
            
        } else if (logic->states[BehaviourStateLogic::IS_KICKING]) {
            
            
            //XXX: send to NUbugger

            //std::cout << "Watching The Ball" << std::endl;
            
            watchTheBall( logic, movement, head);
            
        } else if (logic->states[BehaviourStateLogic::IS_GETTING_UP] or
                   logic->states[BehaviourStateLogic::JUST_GOT_UP] or
                   logic->states[BehaviourStateLogic::JUST_PUT_DOWN] or 
                   logic->states[BehaviourStateLogic::JUST_UNPENALISED]) {
            
            //XXX: send to NUbugger

            //std::cout << "Looking For Landmarks" << std::endl;
            
            doFieldLocalisation( logic, movement, head);            
        } else if (logic->states[BehaviourStateLogic::BALL_IS_LOST] or
                    Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeSinceLastSeen() > 90. //and //XXX: hack, remove this

                   //not logic->states[BehaviourStateLogic::TEAM_SEES_BALL]
                   ) {
            
            //XXX: send to NUbugger

            //std::cout << "Looking For Ball" << std::endl;
            
            doBallLocalisation( logic, movement, head);
            
        } else if ((logic->states[BehaviourStateLogic::IS_APPROACHING_BALL] and
                   not logic->states[BehaviourStateLogic::IS_FURTHEST_FROM_BALL]) or
                   logic->states[BehaviourStateLogic::IS_CLOSEST_TO_BALL] or true) {
            
            //XXX: send to NUbugger

            //std::cout << "In Striker Mode" << std::endl;
            
            doBallApproachAndKick( logic, movement, head);
            
        } else if (logic->states[BehaviourStateLogic::IS_SECOND_FROM_BALL] and
                   not logic->states[BehaviourStateLogic::IS_GOAL_KEEPER]) {
            
            
            //XXX: send to NUbugger

            //std::cout << "In Offensive Support Mode" << std::endl;
            
            //XXX: we need to know if it's not the closest to the ball but is the closest to the offensive position....
            goToOffensiveSupportPosition(logic,  movement, head);
            
        } else if (logic->states[BehaviourStateLogic::IS_SECOND_FROM_BALL] and
                   not logic->states[BehaviourStateLogic::IS_GOAL_KEEPER]) {
            
            
            //XXX: send to NUbugger

            //std::cout << "In Defensive Support Mode" << std::endl;
            
            //XXX: see above
            goToDefensiveSupportPosition( logic,  movement, head);
            
        } else {
            //XXX: send to NUbugger

            //std::cout << "I Am Doing Nothing" << std::endl;
        } /*else if () {
            
        } else if () {
            
        } else if () {
            
        } else if () {
            
        } else if () {
            
        } else if () {
            
        } else if () {
            
        }*/
        
        /*
        //XXX: messy
        if (m_provider->singleChestClick()) {
            while (m_game_info->getCurrentState() != GameInformation::PenalisedState) {
                Blackboard->GameInfo->doManualStateChange();
            }
        } else if (m_provider->doubleChestClick() or m_provider->tripleChestClick()) {
            //XXX: save images
        }*/
        
        //XXX: messy
        if (m_provider->singleChestClick()) {
            if (logic->states[BehaviourStateLogic::GAME_STATE_PENALISED]) {
                while (m_game_info->getCurrentState() != GameInformation::PlayingState)
                    Blackboard->GameInfo->doManualStateChange();
            } else {
                while (m_game_info->getCurrentState() != GameInformation::PenalisedState)
                    Blackboard->GameInfo->doManualStateChange();
            }
            
            if (logic->states[BehaviourStateLogic::GAME_STATE_PENALISED]) {
                Blackboard->Actions->add(NUActionatorsData::Sound, Blackboard->Sensors->GetTimestamp(), "play_soccer.wav");
            } else {
                Blackboard->Actions->add(NUActionatorsData::Sound, Blackboard->Sensors->GetTimestamp(), "penalised.wav");
            }
            
        } else if (m_provider->doubleChestClick() or m_provider->tripleChestClick() or m_provider->longChestClick()) {
            saving_images = not saving_images;
            Blackboard->Jobs->addVisionJob(new SaveImagesJob(saving_images, true));
            
            if (saving_images) {
                Blackboard->Actions->add(NUActionatorsData::Sound, Blackboard->Sensors->GetTimestamp(), "start_saving_images.wav");
            } else {
                Blackboard->Actions->add(NUActionatorsData::Sound, Blackboard->Sensors->GetTimestamp(), "stop_saving_images.wav");
            }
        }
        
        
        
        //update movement and head:
        //std::cout << "about to update movement" << std::endl;
        movement->update();
        //std::cout << "about to update head" << std::endl;
        head->update();
        //std::cout << "Finished Behaviour." << std::endl;
        
        
    };
};

#endif

