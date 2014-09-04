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

#include "ForwardWalkProvider.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"

#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include <math.h>
#include "debug.h"
#include "debugverbositybehaviour.h"



ForwardWalkProvider::ForwardWalkProvider(Behaviour* manager) : BehaviourProvider(manager)
{
}


ForwardWalkProvider::~ForwardWalkProvider()
{
}

void ForwardWalkProvider::doBehaviour()
{
    Blackboard->Jobs->addMotionJob(new WalkJob(1,0,0));
}

