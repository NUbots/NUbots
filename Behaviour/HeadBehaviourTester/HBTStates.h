/*!  @file HBTStates.h
    @brief Provider of Head behaviour Testing behaviour. Darwin simply stands, observes and localises. Modified from Zombie state.
    @author Jake Fountain

    @author Jake Fountain

    Copyright (c) 2012 Jake Fountain

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

#ifndef HBTSTATES_H
#define HBTSTATES_H

#include "Behaviour/BehaviourState.h"
#include "HBTProvider.h"

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

#include "Behaviour/Common/HeadBehaviour.h"

#include "debug.h"

class HBTSubState : public BehaviourState
{
public:
    HBTSubState(HBTProvider* provider){m_provider = provider;}
protected:
    HBTProvider* m_provider;
};

// ----------------------------------------------------------------------------------------------------------------------- PausedState
class HBTState : public HBTSubState
{
public:
    HeadBehaviour* head_behaviour;
    HBTState(HBTProvider* provider) : HBTSubState(provider) {
        head_behaviour = HeadBehaviour::getInstance();
    }
    BehaviourState* nextState() {return m_provider->m_state;}
    void doState()
    {
        while (m_game_info->getCurrentState() != GameInformation::PlayingState)
            m_game_info->doManualStateChange();

        NUActionatorsData* m_actions = Blackboard->Actions;
        
        // the std::vectors are all static since they are used often and we wish to reduce memory operations.
        static std::vector<float> joints(20, 0.0f);            // All joints
        static std::vector<float> nu_nextLeftArmJoints(m_actions->getSize(NUActionatorsData::LArm), 0.0f);   // Left Arm
        static std::vector<float> nu_nextRightArmJoints(m_actions->getSize(NUActionatorsData::RArm), 0.0f);  // Right Arm
        static std::vector<float> nu_nextLeftLegJoints(m_actions->getSize(NUActionatorsData::LLeg), 0.0f);   // Left Leg
        static std::vector<float> nu_nextRightLegJoints(m_actions->getSize(NUActionatorsData::RLeg), 0.0f);  // Right Leg
        static std::vector<float> nu_nextHeadJoints(m_actions->getSize(NUActionatorsData::Head), 0.0f);  // Right Leg


        // Assign the values to each effector.
        nu_nextHeadJoints.assign(joints.begin(), joints.begin()+2);
        //nu_nextLeftArmJoints.assign(joints.begin()+2, joints.begin()+5);
        //nu_nextRightArmJoints.assign(joints.begin()+5, joints.begin()+8);
        nu_nextLeftLegJoints.assign(joints.begin()+8, joints.begin()+14);
        nu_nextRightLegJoints.assign(joints.begin()+14, joints.begin()+20);
        
        //HEAD TRACK
        //if (m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].isObjectVisible())
        //    m_jobs->addMotionJob(new HeadTrackJob(m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL]));
        //else if (m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeSinceLastSeen() > 250)
        //    m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::BallAndLocalisation));
        //UPDATE HEAD
        m_actions->add(NUActionatorsData::Head, Blackboard->Sensors->GetTimestamp()+6000, nu_nextHeadJoints, 0);

        //UPDATE ARMS:
       // m_actions->add(NUActionatorsData::RArm, Blackboard->Sensors->GetTimestamp()+6000, nu_nextRightArmJoints, 30);
        //m_actions->add(NUActionatorsData::LArm, Blackboard->Sensors->GetTimestamp()+6000, nu_nextLeftArmJoints, 30);

        //UPDATE LEGS:
        m_actions->add(NUActionatorsData::RLeg, Blackboard->Sensors->GetTimestamp()+6000, nu_nextRightLegJoints, 65);
        m_actions->add(NUActionatorsData::LLeg, Blackboard->Sensors->GetTimestamp()+6000, nu_nextLeftLegJoints, 65);


        //Head behaviour testing:
        head_behaviour->makeVisionChoice(HeadBehaviour::RLAgentTrainingPolicy);//or pass HeadBehaviour::(M)RLAgentPolicy, HeadBehaviour::TimeVSCostPriority
    };
};

#endif

