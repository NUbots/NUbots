/*! @file GoalKeeperStates.h
    @brief Declaration of the ball is lost states

    @author Josiah Walker
 
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

#ifndef GOAL_KEEPER_STATES_H
#define GOAL_KEEPER_STATES_H

#include "../../SoccerState.h"
#include "GoalKeeperState.h"
#include "Behaviour/BehaviourPotentials.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "Infrastructure/GameInformation/GameInformation.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/ScriptJob.h"
#include "Tools/Math/General.h"

#include "debug.h"
#include "debugverbositybehaviour.h"
using namespace std;

class GoalKeeperSubState : public SoccerState
{
public:
    GoalKeeperSubState(GoalKeeperState* parent) : SoccerState(parent) {};
    virtual ~GoalKeeperSubState() {};
protected:
};

// ----------------------------------------------------------------------------------------------------------------------- BallIsLostPan
/*! @class BallIsLostPan
    In this state we stop and do a wide pan. When the pan is completed we move into the spin state.
 */
class GoalKeeperDives : public GoalKeeperSubState
{
public:
    GoalKeeperDives(GoalKeeperState* parent) : GoalKeeperSubState(parent)
    {
        m_block_time = Blackboard->Sensors->GetTimestamp();
    }
    ~GoalKeeperDives() {};
protected:
    float m_block_time,m_walk_time;
    MotionScript m_script;
    
    BehaviourState* nextState()
    {   // do state transitions in the ball is lost state machine
        return this;
    }
      
    
    void doState()
    {
        // hack it, and put the GameState into Playing
        //while (m_game_info->getCurrentState() != GameInformation::PlayingState)
        //    m_game_info->doManualStateChange();
        
        //Self& self = m_field_objects->self;
        MobileObject& ball = Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL];
        
	    // do the head motion (track ball if visible, pan if not)
	    if (ball.isObjectVisible())
	    {   // track the ball if it is visible
		    m_jobs->addMotionJob(new HeadTrackJob(ball));
		    
		    //do the save if the ball needs it
		    Self& self = Blackboard->Objects->self;
            MobileObject& ball = Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL];
            
            float maxInterceptTime = 7;
            float interceptErrorFraction = 0.5;
            
            //get the relative Y intercept of the object to me (0,0)
            // (ie how far to either side the ball will go)
            Vector2<float> ballVelocity = ball.getEstimatedVelocity();
            Vector2<float> ballVelocityError = ball.getEstimatedVelocityError();
            Vector2<float> ballPosition = ball.getEstimatedFieldLocation(); 
            Vector2<float> ballPositionError = ball.getEstimatedFieldLocationError();
            //cout << "Ball Position: " << self.wmX() << ", " << self.wmY() << endl;
            
            float velocityErrorMagnitude = sqrt(ballVelocityError.x*ballVelocityError.x+ballVelocityError.y*ballVelocityError.y);
            float velocityMagnitude = sqrt(ballVelocity.x*ballVelocityError.x+ballVelocityError.y*ballVelocityError.y);
            float positionErrorMagnitude = sqrt(ballPositionError.x*ballPositionError.x+ballPositionError.y*ballPositionError.y);
            float positionMagnitude = sqrt(ballPosition.x*ballPosition.x+ballPosition.y*ballPosition.y);
            
            //transform from worldspace to robot space
            Vector2<float> relativeBallPosition;
            relativeBallPosition.x = (ballPosition.x-self.wmX())*cos(self.Heading())-(ballPosition.y-self.wmY())*sin(self.Heading());
            relativeBallPosition.y = (ballPosition.x-self.wmX())*sin(self.Heading())+(ballPosition.y-self.wmY())*cos(self.Heading());

            Vector2<float> relativeBallVelocity;
            relativeBallVelocity.x = ballVelocity.x*cos(self.Heading())-ballVelocity.y*sin(self.Heading());
            relativeBallVelocity.y = ballVelocity.x*sin(self.Heading())+ballVelocity.y*cos(self.Heading());
            
            cout << "Relative X pos: " << relativeBallPosition.x << ", Relative X vel: " << relativeBallVelocity.x << endl;
            //cout << velocityErrorMagnitude << " position:" << positionErrorMagnitude << endl;
            
            
            //check the ball is heading towards us
            if (relativeBallPosition.x > 0. and relativeBallVelocity.x < -15. and ball.TimeSeen() > 800) {
                
                //calculate intercept time
                float interceptTime = relativeBallPosition.x/(-relativeBallVelocity.x);
                
                //calculate the Y intercept
                float interceptY = interceptTime*relativeBallVelocity.y+relativeBallPosition.y;
                
                //cout << "Intercept Time: " << interceptTime << ", Intercept Y: " << interceptY << endl;
                
                //cout << "Intercept Time: " << interceptTime << ", Intercept Y: " << interceptY << endl;
                
                //XXX: parameterize defensive area size - this is set to goalsize (*errorFraction for uncertainty)
                float defensiveArea = 150.;
                if (interceptTime < maxInterceptTime and interceptTime > 0.2 and fabs(interceptY) < defensiveArea*(1.+interceptErrorFraction)/2.) {
                    
                    //we have decided we should intercept, now check what move to use
                    float standingBlockHalfSize = 12.5; //width of the robot on the half it is defending
                    float standingSideBlockHalfSize = 18.; //width of the robot on the half it is defending

                    cout << "Intercept Time: " << interceptTime << ", Intercept Y: " << interceptY << endl;
                    cout << "Relative X pos: " << relativeBallPosition.x << ", Relative X vel: " << relativeBallVelocity.x << endl;
                    cout << velocityErrorMagnitude << " position:" << positionErrorMagnitude << " time ball seen: " << ball.TimeSeen() << endl;
                    
                    string blockSide;
                    if (interceptY > 0) {
                        blockSide = "Left";
                    } else {
                        blockSide = "Right";
                    }

                    string blockType;
                    if (fabs(interceptY) < standingBlockHalfSize) {
                        blockType = "StandingBlock";
                    } else if (fabs(interceptY) < standingBlockHalfSize) {
                        blockType = "SideBlock";
                    } else {
                        blockType = "DiveBlock";
                    }
                    
                    cout << "Saving!" << endl;
                    ScriptJob* currentSave = new ScriptJob(m_data->CurrentTime+10, "Save"+blockSide); //blockType+blockSide);
                    //m_script.play(m_data,m_actions);
                    m_jobs->addMotionJob(currentSave);
                    m_block_time = Blackboard->Sensors->GetTimestamp()+1000;
                    //return true;
                }
            }
	    }
	    else if (ball.TimeSinceLastSeen() > 250 and ball.TimeSinceLastSeen() < 3000)
	    {   // pan for the ball if it hasn't been seen for a bit
		    m_jobs->addMotionJob(new HeadPanJob(ball));
	    }
	    else
		    m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::BallAndLocalisation));
	
        if (Blackboard->Sensors->GetTimestamp() > m_block_time) {
		    Blackboard->Jobs->addMotionJob(new WalkJob(0,0,0));
	    }
    }

private:
    void reset()
    {
        m_block_time = Blackboard->Sensors->GetTimestamp();
    }
};



#endif

