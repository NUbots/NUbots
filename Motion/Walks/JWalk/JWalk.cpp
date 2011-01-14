/*! @file JWalk.cpp
    @brief Implementation of jwalk class

    @author Jason Kulk
 
 Copyright (c) 2009, 2010, 2011 Jason Kulk
 
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

#include "JWalk.h"
#include "JWalkState.h"

#include "JWalkStance.h"
#include "JWalkPush.h"
#include "JWalkSwing.h"
#include "JWalkAccept.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "debug.h"
#include "debugverbositynumotion.h"

JWalk* JWalkBlackboard = 0;

JWalk::JWalk() : NUWalk(Blackboard->Sensors, Blackboard->Actions)
{
    JWalkBlackboard = this;
    
    GaitPhase = 0;
    GaitFrequency = 0.6;
    
    LeftStance = new JWalkStance(NUData::LLeg);
    LeftPush = new JWalkPush(NUData::LLeg);
    LeftSwing = new JWalkSwing(NUData::LLeg);
    LeftAccept = new JWalkAccept(NUData::LLeg);
    
    RightStance = new JWalkStance(NUData::RLeg);
    RightPush = new JWalkPush(NUData::RLeg);
    RightSwing = new JWalkSwing(NUData::RLeg);
    RightAccept = new JWalkAccept(NUData::RLeg);
    
    LeftState = LeftStance;
    RightState = RightStance;
    
    CurrentTime = 0;
    PreviousTime = 0;
    
    m_walk_parameters.load("JWalkStart");
    // Initialise the leg values
    m_initial_lleg = vector<float>(Blackboard->Actions->getSize(NUActionatorsData::LLeg), 0);
    m_initial_rleg = vector<float>(Blackboard->Actions->getSize(NUActionatorsData::RLeg), 0);
    
    // Initialise the arm values
    float larm[] = {0.1, 1.57, 0.15, -1.57};
    float rarm[] = {-0.1, 1.57, 0.15, 1.57};
    m_initial_larm = vector<float>(larm, larm + sizeof(larm)/sizeof(*larm));
    m_initial_rarm = vector<float>(rarm, rarm + sizeof(rarm)/sizeof(*rarm));
}

/*! @brief Destructor for motion module
 */
JWalk::~JWalk()
{
    delete LeftStance;
    delete LeftPush;
    delete LeftSwing;
    delete LeftAccept;
    delete RightStance;
    delete RightPush;
    delete RightSwing;
    delete RightAccept;
}

void JWalk::doWalk()
{
    CurrentTime = Blackboard->Sensors->CurrentTime;
    calculateGaitPhase();
    
    LeftState = LeftState->next();
    RightState = RightState->next();
    
    #if DEBUG_NUMOTION_VERBOSITY > 0
        debug << "JWalk::doWalk(). CurrentTime: " << CurrentTime << " Phase: " << GaitPhase << " Left: " << LeftState->Name << " Right: " << RightState->Name << endl;
    #endif
    
    LeftState->doIt();
    RightState->doIt();
    
    PreviousTime = CurrentTime;
}

void JWalk::calculateGaitPhase()
{
    double dt = (CurrentTime - PreviousTime)/1000;
    if (dt < 0.2)
    {
        if (m_target_speed_x != 0 or m_target_speed_y != 0 or m_target_speed_yaw != 0)
        {	// only progress the GaitPhase if we have a non-zero speed
            GaitPhase += dt*GaitFrequency;
            if (GaitPhase >= 1)
                GaitPhase--;
        }
    }
    else
    {	// reset the gait phase if there has been a 'gap'
        GaitPhase = 0;
        LeftState = LeftStance;
        RightState = RightStance;
    }
}


