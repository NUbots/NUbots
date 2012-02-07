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

#ifndef ZOMBIESTATES_H
#define ZOMBIESTATES_H

#include "Behaviour/BehaviourState.h"
#include "ZombieProvider.h"

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

#include "debug.h"

class ZombieSubState : public BehaviourState
{
public:
    ZombieSubState(ZombieProvider* provider){m_provider = provider;};
protected:
    ZombieProvider* m_provider;
};

// ----------------------------------------------------------------------------------------------------------------------- PausedState
class ZombieState : public ZombieSubState
{
public:
    ZombieState(ZombieProvider* provider) : ZombieSubState(provider) {};
    BehaviourState* nextState() {return m_provider->m_state;};
    void doState()
    {
        NUActionatorsData* m_actions = Blackboard->Actions;
        
        // the vectors are all static since they are used often and we wish to reduce memory operations.
        static std::vector<float> joints(20, 0.0f);            // All joints
        static vector<float> nu_nextLeftArmJoints(m_actions->getSize(NUActionatorsData::LArm), 0.0f);   // Left Arm
        static vector<float> nu_nextRightArmJoints(m_actions->getSize(NUActionatorsData::RArm), 0.0f);  // Right Arm
        static vector<float> nu_nextLeftLegJoints(m_actions->getSize(NUActionatorsData::LLeg), 0.0f);   // Left Leg
        static vector<float> nu_nextRightLegJoints(m_actions->getSize(NUActionatorsData::RLeg), 0.0f);  // Right Leg
        static vector<float> nu_nextHeadJoints(m_actions->getSize(NUActionatorsData::Head), 0.0f);  // Right Leg


        // Assign the values to each effector.
        nu_nextHeadJoints.assign(joints.begin(), joints.begin()+2);
        nu_nextLeftArmJoints.assign(joints.begin()+2, joints.begin()+5);
        nu_nextRightArmJoints.assign(joints.begin()+5, joints.begin()+8);
        nu_nextLeftLegJoints.assign(joints.begin()+8, joints.begin()+14);
        nu_nextRightLegJoints.assign(joints.begin()+14, joints.begin()+20);
        
        //UPDATE HEAD
        m_actions->add(NUActionatorsData::Head, Blackboard->Sensors->GetTimestamp()+6000, nu_nextHeadJoints, 30);

        //UPDATE ARMS:
        m_actions->add(NUActionatorsData::RArm, Blackboard->Sensors->GetTimestamp()+6000, nu_nextRightArmJoints, 30);
        m_actions->add(NUActionatorsData::LArm, Blackboard->Sensors->GetTimestamp()+6000, nu_nextLeftArmJoints, 30);

        //UPDATE LEGS:
        m_actions->add(NUActionatorsData::RLeg, Blackboard->Sensors->GetTimestamp()+6000, nu_nextRightLegJoints, 65);
        m_actions->add(NUActionatorsData::LLeg, Blackboard->Sensors->GetTimestamp()+6000, nu_nextLeftLegJoints, 65);
    };
};

#endif

