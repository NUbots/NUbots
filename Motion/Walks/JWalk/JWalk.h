/*! @file JWalk.h
    @brief Declaration of the JWalk
 
    @class JWalk
    @brief Jason's walk engine
 
 	JWalk is a modular walk engine designed for high speed locomotion.
 	At its core is a four-state state machine; STANCE, PUSH, SWING, ACCEPT.
 	Each of these states functions independently of the others.
 
 	Data is shared using the JWalkBlackboard.
 
 	STANCE. In the stance state the engine needs to maintain balance,
 		    and use the leg to move the robot in the desired direction of travel.
 			For walking, the transition from STANCE to PUSH happens when the other foot starts
 			its ACCEPT. However, for running and sprinting the transition is much sooner.
 
 	PUSH. 	In the push state the engine needs to transfer weight to the ACCEPT foot.
            For all locomotion, the transition from PUSH to SWING occurs when most of
 			weight has left this foot.
 
 	SWING. 	In the swing state the engine needs to move the leg in the desired direction of travel.
 			To maintain balance the swinging leg might need to move in a different direction. 
 			We transition to the ACCEPT state when the swing is finished, or when the foot contacts
 			the ground.
 
 	ACCEPT. In the accept state we just restore balance. We transition to the STANCE state when
 			this is achieved.
 
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

#ifndef JWALK_H
#define JWALK_H

#include "Motion/NUWalk.h"
class JWalkState;

class JWalk : public NUWalk
{
public:
    JWalk();
    ~JWalk();
protected:
    void doWalk();
private:
    void updateJWalkBlackboard();
    void calculateGaitPhase();
public:
    // The JWalk blackboard 
    // Implementing a blackboard here is perfectly safe because nobody knows how to cast a NUWalk to a JWalk,
    // and therefore, nobody can access these variables
    float WalkForwardSpeed;				//!< the current forward walk speed in cm/s (forward is +ve)
    float WalkSidewardSpeed;			//!< the current sideward walk speed in cm/s (left is +ve)
    float WalkTurnSpeed;				//!< the current turn walk speed in rad/s (left is +ve)
    float WalkFrequency;				//!< the current walk frequency in Hz (1.0Hz means one left and one right step in 1 second)
    
    WalkParameters* Parameters;			//!< the current set of walk parameters
    
    bool LArmEnabled;					//!< true if we have control of the left arm
    bool RArmEnabled;					//!< true if we have control of the right arm
    
    float GaitPhase;					//!< the current gait phase 0->1 (possibly redundant with this engine)
    JWalkState* LeftState;				//!< the current state the left leg is in
    JWalkState* RightState;				//!< the current state the right leg is in
    
	JWalkState* LeftStance;
    JWalkState* LeftPush;
    JWalkState* LeftSwing;
	JWalkState* LeftAccept;
    
    JWalkState* RightStance;
    JWalkState* RightPush;
    JWalkState* RightSwing;
    JWalkState* RightAccept;
    
    double CurrentTime;
    double PreviousTime;

};

extern JWalk* JWalkBlackboard;

#endif

