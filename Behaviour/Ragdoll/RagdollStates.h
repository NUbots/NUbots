/*! @file ChaseBallStates.h
    @brief Chase ball states

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

#ifndef RAGDOLLSTATES_H
#define RAGDOLLSTATES_H

#include "Behaviour/BehaviourState.h"
#include "Behaviour/Common/HeadBehaviour.h"
#include "RagdollProvider.h"

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

class RagdollSubState : public BehaviourState
{
public:
    RagdollSubState(RagdollProvider* provider){m_provider = provider;}
protected:
    RagdollProvider* m_provider;
};

// ----------------------------------------------------------------------------------------------------------------------- PausedState
class RagdollState : public RagdollSubState
{
public:
    RagdollState(RagdollProvider* provider) : RagdollSubState(provider) {}
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
        nu_nextLeftArmJoints.assign(joints.begin()+2, joints.begin()+5);
        nu_nextRightArmJoints.assign(joints.begin()+5, joints.begin()+8);
        //UPDATE HEAD
        m_actions->add(NUActionatorsData::Head, Blackboard->Sensors->GetTimestamp()+6000, nu_nextHeadJoints, 0);

        //UPDATE ARMS:
        m_actions->add(NUActionatorsData::RArm, Blackboard->Sensors->GetTimestamp()+6000, nu_nextRightArmJoints, 0);
        m_actions->add(NUActionatorsData::LArm, Blackboard->Sensors->GetTimestamp()+6000, nu_nextLeftArmJoints, 0);

        //UPDATE LEGS:
        m_actions->add(NUActionatorsData::RLeg, Blackboard->Sensors->GetTimestamp()+6000, nu_nextRightLegJoints, 0);
        m_actions->add(NUActionatorsData::LLeg, Blackboard->Sensors->GetTimestamp()+6000, nu_nextLeftLegJoints, 0);
    };
};

#endif

