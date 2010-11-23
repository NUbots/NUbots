/*! @file DribblingChallengeProvider.cpp
    @brief Implementation of dribbling challenge behaviour class

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

#include "DribblingChallengeProvider.h"
#include "DribblingChallengeStates.h"

#include "Behaviour/Jobs/JobList.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Vision/FieldObjects/FieldObjects.h"
#include "Behaviour/TeamInformation.h"
#include "Behaviour/GameInformation.h"

#include "Behaviour/Jobs/MotionJobs/HeadJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadPanJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadNodJob.h"
#include "Behaviour/Jobs/MotionJobs/WalkJob.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

using namespace std;

DribblingChallengeProvider::DribblingChallengeProvider(Behaviour* manager) : BehaviourFSMProvider(manager)
{
    m_paused = new DribblingPausedState(this);
    m_move_to_ball = new DribblingMoveToBallState(this);
    m_scan_for_obstacles = new DribblingScanForObstaclesState(this);
    m_kicking = new DribblingKickingState(this);
    m_ball_lost = new DribblingBallLostState(this);
    m_lost = new DribblingImLostState(this);
    
    m_state = m_paused;
}


DribblingChallengeProvider::~DribblingChallengeProvider()
{
    delete m_paused;
    delete m_move_to_ball;
    delete m_scan_for_obstacles;
    delete m_kicking;
    delete m_ball_lost;
    delete m_lost;
}

BehaviourState* DribblingChallengeProvider::nextStateCommons()
{
    MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
    GameInformation::RobotState game_state = m_game_info->getCurrentState();
    if (game_state != GameInformation::PlayingState)
        return m_paused;
    else if (m_field_objects->self.lost())
        return m_lost;
    else if (ball.lost() and m_state != m_scan_for_obstacles)
        return m_ball_lost;
    else if (m_state == m_ball_lost)
        return m_move_to_ball;
    else
        return m_state;
}


