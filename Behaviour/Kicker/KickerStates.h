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

#ifndef KICKERSTATES_H
#define KICKERSTATES_H

#include "Behaviour/BehaviourState.h"
#include "KickerProvider.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"

#include "Infrastructure/Jobs/MotionJobs/KickJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"
#include "Infrastructure/Jobs/MotionJobs/MotionFreezeJob.h"
#include "NUPlatform/NUActionators/NUSounds.h"

#include "debug.h"

class KickerState : public BehaviourState
{
public:
    KickerState(KickerProvider* provider){m_provider = provider;};
protected:
    KickerProvider* m_provider;
};

// ----------------------------------------------------------------------------------------------------------------------- WaitState
class WaitState : public KickerState
{
public:
    int m_initialMoveCounter;
    WaitState(KickerProvider* provider) : KickerState(provider) {m_initialMoveCounter = 0;};
    BehaviourState* nextState()
    {
        return m_provider->m_state;
    };
    
    void doState()
    {
        if(m_initialMoveCounter < 10)
        {
            m_jobs->addMotionJob(new WalkJob(0.001,0,0));
            m_initialMoveCounter++;
        }
        else
        {
            m_jobs->addMotionJob(new WalkJob(0,0,0));
        }
    };
};

// ----------------------------------------------------------------------------------------------------------------------- KickState
class KickState : public KickerState
{
public:
    KickState(KickerProvider* provider) : KickerState(provider)
    {
        m_kickActivePrev = false;
        m_kickPos = vector<float>(2,0);
        m_kickTarget = vector<float>(2,0);
    };
    BehaviourState* nextState()
    {
        bool kickActive = false;
        m_provider->m_data->get(NUSensorsData::MotionKickActive, kickActive);
        bool kickFinished = m_kickActivePrev && !kickActive;
        m_kickActivePrev = kickActive;

        if (kickFinished)
        {
            debug << "Kicking -> Waiting" << endl;
            return m_provider->m_wait_state;
        }
        else
            return m_provider->m_state;
    };
    
    void doState()
    {

        // left
        if(true)
        {
            m_kickPos[0] = 10.0;
            m_kickPos[1] = 5.0;
            m_kickTarget[0] = 100.0;
            m_kickTarget[1] = 5.0;
        }
        // right
        else if(true)
        {
            m_kickPos[0] = 10.0;
            m_kickPos[1] = -5.0;
            m_kickTarget[0] = 100.0;
            m_kickTarget[1] = -5.0;
        }
        // left side
        else if(true)
        {
            m_kickPos[0] = 10.0;
            m_kickPos[1] = 5.0;
            m_kickTarget[0] = 10.0;
            m_kickTarget[1] = 100.0;
        }
        // right side
        else if(true)
        {
            m_kickPos[0] = 10.0;
            m_kickPos[1] = -5.0;
            m_kickTarget[0] = 10.0;
            m_kickTarget[1] = -100.0;
        }

        KickJob* kick = new KickJob(0,m_kickPos,m_kickTarget);
        m_provider->m_jobs->addMotionJob(kick);
    };
private:
    bool m_kickActivePrev;
    vector<float> m_kickPos;
    vector<float> m_kickTarget;
};

#endif

