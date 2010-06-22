/*! @file PassingChallengeProvider.cpp
    @brief Implementation of chase ball behaviour class

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

#include "PassingChallengeProvider.h"
#include "PassingChallengeStates.h"

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

PassingChallengeProvider::PassingChallengeProvider(Behaviour* manager) : BehaviourFSMProvider(manager)
{
    m_position_state = new PassingPositionState(this);
    m_kick_state = new PassingKickingState(this);
    m_ball_lost_state = new PassingBallLostState(this);
    m_lost_state = new PassingImLostState(this);
    m_paused_state = new PassingPausedState(this);
    
    m_state = m_lost_state;
}


PassingChallengeProvider::~PassingChallengeProvider()
{
    delete m_position_state;
    delete m_kick_state;
    delete m_ball_lost_state;
    delete m_lost_state;
}

BehaviourState* PassingChallengeProvider::nextStateCommons()
{
    MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
    GameInformation::RobotState game_state = m_game_info->getCurrentState();
    if (game_state != GameInformation::PlayingState)
        return m_paused_state;
    else if (m_field_objects->self.lost())
        return m_lost_state;
    else if (ball.lost())
        return m_ball_lost_state;
    else
    {
        if (m_game_info->getPlayerNumber() == 2)
        {
            if (ball.X() < 0)
                return m_kick_state;
            else
                return m_position_state;
        }
        else 
        {
            if (ball.X() >= 0)
                return m_kick_state;
            else
                return m_position_state;
        }
    }
        
    return m_state;
}


