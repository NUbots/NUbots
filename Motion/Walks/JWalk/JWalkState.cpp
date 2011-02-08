/*! @file JWalkState.h
    @brief Declaration of a jwalk state
 
    @class JWalkState
    @brief An abstract jwalk state
 
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

#include "JWalkState.h"

/*! @brief Constructor for a JWalkState
 	@param name the name of the state
 	@param leg either NUData::LLeg or NUData::RLeg which this state will control
 */
JWalkState::JWalkState(const string& name, const NUData::id_t& leg) 
{
    m_name = leg.Name + name;
    m_leg = leg;
    if (m_leg == NUData::LLeg)
    {
        m_other_leg = NUData::RLeg;
        m_state = &JWalkBlackboard->LeftState;
        m_stance = &JWalkBlackboard->LeftStance;
        m_push = &JWalkBlackboard->LeftPush;
        m_swing = &JWalkBlackboard->LeftSwing;
        m_accept = &JWalkBlackboard->LeftAccept;
        m_other_state = &JWalkBlackboard->RightState;
        m_other_stance = &JWalkBlackboard->RightStance;
        m_other_push = &JWalkBlackboard->RightPush;
        m_other_swing = &JWalkBlackboard->RightSwing;
        m_other_accept = &JWalkBlackboard->RightAccept;
    }
    else
    {
        m_other_leg = NUData::LLeg;
        m_state = &JWalkBlackboard->RightState;
        m_stance = &JWalkBlackboard->RightStance;
        m_push = &JWalkBlackboard->RightPush;
        m_swing = &JWalkBlackboard->RightSwing;
        m_accept = &JWalkBlackboard->RightAccept;
        m_other_state = &JWalkBlackboard->LeftState;
        m_other_stance = &JWalkBlackboard->LeftStance;
        m_other_push = &JWalkBlackboard->LeftPush;
        m_other_swing = &JWalkBlackboard->LeftSwing;
        m_other_accept = &JWalkBlackboard->LeftAccept;
    }
}

/*! @brief Returns the state's name */
string& JWalkState::getName()
{
    return m_name;
}