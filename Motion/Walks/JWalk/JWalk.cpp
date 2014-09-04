/*! @file JWalk.cpp
    @brief Implementation of the JWalk

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
    m_walk_parameters.load("JWalkStart");
    
    WalkForwardSpeed = 0;
    WalkSidewardSpeed = 0;
    WalkTurnSpeed = 0;
    WalkFrequency = 0.6;			// a constant for now
    
    Parameters = &m_walk_parameters;
    
    LArmEnabled = false;
    RArmEnabled = false;
    
    LeftStance = new JWalkStance(NUData::LLeg);
    LeftPush = new JWalkPush(NUData::LLeg);
    LeftSwing = new JWalkSwing(NUData::LLeg);
    LeftAccept = new JWalkAccept(NUData::LLeg);
    
    RightStance = new JWalkStance(NUData::RLeg);
    RightPush = new JWalkPush(NUData::RLeg);
    RightSwing = new JWalkSwing(NUData::RLeg);
    RightAccept = new JWalkAccept(NUData::RLeg);
    
    GaitPhase = 0;
    LeftState = LeftStance;
    RightState = RightStance;
    
    CurrentTime = 0;
    PreviousTime = 0;
    
    
    // Initialise the leg values
    m_initial_lleg = std::vector<float>(Blackboard->Actions->getSize(NUActionatorsData::LLeg), 0);
    m_initial_rleg = std::vector<float>(Blackboard->Actions->getSize(NUActionatorsData::RLeg), 0);
    
    // Initialise the arm values
    float larm[] = {0.1, 1.57, 0.15, -1.57};
    float rarm[] = {-0.1, 1.57, 0.15, 1.57};
    m_initial_larm = std::vector<float>(larm, larm + sizeof(larm)/sizeof(*larm));
    m_initial_rarm = std::vector<float>(rarm, rarm + sizeof(rarm)/sizeof(*rarm));
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

    updateJWalkBlackboard();
    calculateGaitPhase();
    
    JWalkState* next_l, *next_r;		// to avoid the problem of having one state change before the other
    next_l = LeftState->next();			// has decided which state is next, we copy the result of next()
    next_r = RightState->next();		// into a buffer, and set both at the same time
	LeftState = next_l;
    RightState = next_r;
    
    #if DEBUG_NUMOTION_VERBOSITY > 0
        debug << "JWalk::doWalk(). CurrentTime: " << CurrentTime << " Phase: " << GaitPhase << " Left: " << LeftState->getName() << " Right: " << RightState->getName() << std::endl;
    #endif
    
    LeftState->doIt();
    RightState->doIt();
    
    PreviousTime = CurrentTime;
}

/*! @brief Updates parts of the JWalkBlackboard that are copies of data stored elsewhere.
           In particular, it copies things from the NUWalk and WalkParameters.
 */
void JWalk::updateJWalkBlackboard()
{
    WalkForwardSpeed = m_speed_x;
    WalkSidewardSpeed = m_speed_y;
    WalkTurnSpeed = m_speed_yaw;
    
    // m_walk_parameters (will copy all of the parameters into there local copies)
    WalkFrequency = 0.6;			// a constant for now
    
    LArmEnabled = m_larm_enabled;
    RArmEnabled = m_rarm_enabled;
}

/*! @brief Calculates the GaitPhase. The GaitPhase is not incremented while we are just standing still wanting for non-zero speed
 	       The GaitPhase and LeftState/RightState are automatically reset if there is a long gap,
           ie control of the robot has been taken away from the walk.
 */
void JWalk::calculateGaitPhase()
{
    double dt = (CurrentTime - PreviousTime)/1000;
    if (dt < 0.2)
    {
        if (not (m_target_speed_x == 0 and m_target_speed_y == 0 and m_target_speed_yaw == 0 and LeftState == LeftStance and RightState == RightStance))
        {	// only progress the GaitPhase when the speed is non-zero and we are not just standing still
            GaitPhase += dt*WalkFrequency;
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


