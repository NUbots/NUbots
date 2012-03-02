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
#include "Kinematics/NAOInverseKinematics.h"
using namespace std;

IKTestProvider::IKTestProvider(Behaviour* manager) : BehaviourProvider(manager),m_current_ik_motion(NULL)
{
    m_ik = new NAOInverseKinematics();
}


IKTestProvider::~IKTestProvider()
{
    delete m_ik;
    delete m_current_ik_motion;
}

void IKTestProvider::doBehaviour()
{
    if(!m_current_ik_motion)
    {
        Ik_squat* temp = new Ik_squat();
        temp->initialiseTiming(Blackboard->Sensors->CurrentTime, 4000, 0);
        m_prev_time = Blackboard->Sensors->CurrentTime;
        temp->initialiseSquat(50, 40, 0, 100);
        m_current_ik_motion = temp;
        std::cout << "motion intialised" << std::endl;
    }
    double elapsed = Blackboard->Sensors->CurrentTime - m_prev_time;
    m_prev_time = Blackboard->Sensors->CurrentTime;

    std::cout << "time elapsed: " << elapsed << std::endl;
    if(m_current_ik_motion->seek(elapsed))
    {
        std::cout << "Getting Matrix..." << std::endl;
        Matrix temp = m_current_ik_motion->leftFootPose();
        std::cout << "Left: " << std::endl << temp << std::endl;
    }
}

