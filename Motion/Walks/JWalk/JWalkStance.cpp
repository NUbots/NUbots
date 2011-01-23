/*! @file JWalkStance.cpp
    @brief Implementation of a jwalk's stance state
 
    @author Jason Kulk
 
  Copyright (c) 2010, 2011 Jason Kulk
 
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

#include "JWalkStance.h"

JWalkStance::JWalkStance(const NUData::id_t& leg) : JWalkState("Stance", leg)
{
}

JWalkStance::~JWalkStance()
{
}

void JWalkStance::doIt()
{
}


/*! @brief Returns the next walk state
 	
 	If the desired walk speed is 0, then we force staying in the stance state, to bring the robot to a stop.
 	If we are standing, then we start by moving to either push or accept
 	If we have finished the stance state, then move to the push state
 */
JWalkState* JWalkStance::next()
{
    if (JWalkBlackboard->WalkForwardSpeed == 0 and JWalkBlackboard->WalkSidewardSpeed == 0 and JWalkBlackboard->WalkTurnSpeed == 0)
    {	// if the desired speed is zero we stay in stance
    	return this;
    }
    else if (JWalkBlackboard->LeftState == JWalkBlackboard->LeftStance and JWalkBlackboard->RightState == JWalkBlackboard->RightStance)
    {	// if we are standing, and we now have a non-zero speed; its time to start walking
        float thisforce, otherforce;
        if (Blackboard->Sensors->getForce(m_leg, thisforce) and Blackboard->Sensors->getForce(m_other_leg, otherforce))
        {	// a nice quirk is to have the foot with the least amount of weight on it lift first
            if (thisforce < otherforce)
                return *m_push;
            else
                return *m_accept;
        }
        else
        {	// if I don't have foot sensors then the left foot always pushes
            if (m_leg == NUData::LLeg)
                return *m_push;
            else
                return *m_accept;
        }
    }
    else if (false)	// TODO: Need to implement the natural stopping condition for the stance state here
    {	// if we are finished stance then go to the push
        return *m_push;
    }
    else
        return this;
}

