/*! @file VisionCalibrationProvider.cpp
    @brief Implementation of Pose behaviour class

    @author Aaron Wong
 
 Copyright (c) 2010 Aaron Wong
 
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

#include "IKTestProvider.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"

#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"

#include <math.h>
#include "debug.h"
#include "debugverbositybehaviour.h"
#include "Kinematics/Ik_squat.h"
#include "Kinematics/Ik_hula.h"
#include "Kinematics/NAOInverseKinematics.h"
using namespace std;

IKTestProvider::IKTestProvider(Behaviour* manager) : BehaviourProvider(manager),m_current_ik_motion(NULL)
{
    m_ik = new NAOInverseKinematics();
    m_sequence_count = 0;
}


IKTestProvider::~IKTestProvider()
{
    delete m_ik;
    delete m_current_ik_motion;
}

void IKTestProvider::doBehaviour()
{
    NUActionatorsData* actions = Blackboard->Actions;

    static std::vector<float> target_joints(actions->getSize(NUActionatorsData::All), 0.0f);
    static std::vector<float> left_leg_joints(actions->getSize(NUActionatorsData::LLeg), 0.0f);
    static std::vector<float> right_leg_joints(actions->getSize(NUActionatorsData::RLeg), 0.0f);
    static std::vector<float> left_arm_joints(actions->getSize(NUActionatorsData::LArm), 0.0f);
    static std::vector<float> right_arm_joints(actions->getSize(NUActionatorsData::RArm), 0.0f);

    static const std::vector<float>::iterator head_start = target_joints.begin();
    static const std::vector<float>::iterator head_end = head_start + actions->getSize(NUActionatorsData::Head);

    static const std::vector<float>::iterator left_arm_start = head_end;
    static const std::vector<float>::iterator left_arm_end = left_arm_start + actions->getSize(NUActionatorsData::LArm);

    static const std::vector<float>::iterator right_arm_start = left_arm_end;
    static const std::vector<float>::iterator right_arm_end = right_arm_start + actions->getSize(NUActionatorsData::RArm);

    static const std::vector<float>::iterator left_leg_start = right_arm_end;
    static const std::vector<float>::iterator left_leg_end = left_leg_start + actions->getSize(NUActionatorsData::LLeg);

    static const std::vector<float>::iterator right_leg_start = left_leg_end;
    static const std::vector<float>::iterator right_leg_end = right_leg_start + actions->getSize(NUActionatorsData::RLeg);

    double elapsed = Blackboard->Sensors->CurrentTime - m_prev_time;
    m_prev_time = Blackboard->Sensors->CurrentTime;

    if(!m_current_ik_motion)
    {
//        Ik_squat* temp = new Ik_squat();
//        temp->initialiseTiming(Blackboard->Sensors->CurrentTime, 4000, 0);
//        temp->initialiseSquat(200, 100, 0.5, 200);

        const unsigned int total_steps = 6;
        unsigned int current_step = m_sequence_count % total_steps;
        switch (current_step)
        {
       case 0:
            {
                Ik_squat* temp = new Ik_squat();
                temp->initialiseTiming(Blackboard->Sensors->CurrentTime, 3000, 5);
                temp->initialiseSquat(200, 100, 200, 0.5);
                m_current_ik_motion = temp;
            }
            break;

        case 1:
            {
            Ik_hula* temp = new Ik_hula();
            temp->initialiseTiming(Blackboard->Sensors->CurrentTime, 2000, 3);
            temp->initialiseHula(170, 150, 0.5, 50);
            m_current_ik_motion = temp;
            }
            break;

        case 2:
            {
            Ik_squat* temp = new Ik_squat();
            temp->initialiseTiming(Blackboard->Sensors->CurrentTime, 2000, 5);
            temp->initialiseSquat(200, 100, 100, 0);
            m_current_ik_motion = temp;
            }
            break;

        case 3:
            {
            Ik_hula* temp = new Ik_hula();
            temp->initialiseTiming(Blackboard->Sensors->CurrentTime, 1500, 3);
            temp->initialiseHula(170, 100, 0.0, 40);
            m_current_ik_motion = temp;
            }
            break;

        case 4:
            {
            Ik_hula* temp = new Ik_hula();
            temp->initialiseTiming(Blackboard->Sensors->CurrentTime, 1500, 3);
            temp->initialiseHula(110, 250, 0.0, 40);
            m_current_ik_motion = temp;
            }
            break;

        case 5:
             {
                 Ik_squat* temp = new Ik_squat();
                 temp->initialiseTiming(Blackboard->Sensors->CurrentTime, 3000, 5);
                 temp->initialiseSquat(150, 100, 300, 1.5);
                 m_current_ik_motion = temp;
             }
             break;
        }

        m_sequence_count++;
        m_moving_to_position = true;
        m_ik->calculateLegJoints(m_current_ik_motion->leftFootPose(), m_current_ik_motion->rightFootPose(), target_joints);

        left_leg_joints.assign(left_leg_start, left_leg_end);
        right_leg_joints.assign(right_leg_start, right_leg_end);

        actions->add(NUActionatorsData::LLeg, m_data->CurrentTime + 1000, left_leg_joints, 75);
        actions->add(NUActionatorsData::RLeg, m_data->CurrentTime + 1000, right_leg_joints, 75);

    }

    if(m_moving_to_position)
    {
        if(inPosition())
            m_moving_to_position = false;
        else
            return;
    }
    else
    {
        m_current_ik_motion->seek(elapsed);
    }


    if(m_current_ik_motion and !m_current_ik_motion->complete())
    {
        m_ik->calculateLegJoints(m_current_ik_motion->leftFootPose(), m_current_ik_motion->rightFootPose(), target_joints);

        left_leg_joints.assign(left_leg_start, left_leg_end);
        right_leg_joints.assign(right_leg_start, right_leg_end);

        left_arm_joints.assign(left_arm_start, left_arm_end);
        right_arm_joints.assign(right_arm_start, right_arm_end);

//        std::cout << "Left: [";
//        for (std::vector<float>::iterator it = left_leg_joints.begin(); it != left_leg_joints.end(); ++it)
//        {
//            if(it != left_leg_joints.begin())
//                std::cout << ",";
//            std::cout << *it;
//        }
//        std::cout << "]" << std::endl;

//        std::cout << "Right: [";
//        for (std::vector<float>::iterator it = right_leg_joints.begin(); it != right_leg_joints.end(); ++it)
//        {
//            if(it != right_leg_joints.begin())
//                std::cout << ", ";
//            std::cout << *it;
//        }
//        std::cout << "]" << std::endl;

        left_arm_joints[0] = 0.5;
        left_arm_joints[1] = 1.5;

        right_arm_joints[0] = -0.5;
        right_arm_joints[1] = 1.5;



        actions->add(NUActionatorsData::LArm, m_data->CurrentTime, left_arm_joints, 75);
        actions->add(NUActionatorsData::RArm, m_data->CurrentTime, right_arm_joints, 75);

        actions->add(NUActionatorsData::LLeg, m_data->CurrentTime, left_leg_joints, 75);
        actions->add(NUActionatorsData::RLeg, m_data->CurrentTime, right_leg_joints, 75);
    }
    else
    {
        delete m_current_ik_motion;
        m_current_ik_motion = NULL;
    }
}

bool IKTestProvider::inPosition()
{
    NUActionatorsData* actions = Blackboard->Actions;
    static std::vector<float> target_joints(actions->getSize(NUActionatorsData::All), 0.0f);
    static std::vector<float> l_leg_targets(actions->getSize(NUActionatorsData::LLeg), 0.0f);
    static std::vector<float> r_leg_targets(actions->getSize(NUActionatorsData::RLeg), 0.0f);
    static std::vector<float> l_leg_positions(actions->getSize(NUActionatorsData::LLeg), 0.0f);
    static std::vector<float> r_leg_positions(actions->getSize(NUActionatorsData::RLeg), 0.0f);
    static std::vector<float>::iterator l_leg_start = target_joints.begin() + actions->getSize(NUActionatorsData::Head) + actions->getSize(NUActionatorsData::LArm) + actions->getSize(NUActionatorsData::RArm);
    static std::vector<float>::iterator r_leg_start = l_leg_start + actions->getSize(NUActionatorsData::LLeg);

    m_ik->calculateLegJoints(m_current_ik_motion->leftFootPose(), m_current_ik_motion->rightFootPose(), target_joints);
    l_leg_targets.assign(l_leg_start, r_leg_start);
    r_leg_targets.assign(r_leg_start, target_joints.end());


    m_data->getPosition(NUSensorsData::LLeg, l_leg_positions);
    m_data->getPosition(NUSensorsData::RLeg, r_leg_positions);

    return mathGeneral::allEqual(l_leg_positions, l_leg_targets, 0.1f) and mathGeneral::allEqual(r_leg_positions, r_leg_targets, 0.1f);
}
