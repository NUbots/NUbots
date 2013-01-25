/*! @file ChaseStates.h
    @brief Declaration of the chase ball states

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

#ifndef CHASE_STATES_H
#define CHASE_STATES_H

#include "../../SoccerState.h"
class SoccerFSMState;       // ChaseState is a SoccerFSMState

#include "Behaviour/BehaviourPotentials.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "Behaviour/Common/HeadBehaviour.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"

#include <algorithm>

#include "debug.h"
#include "debugverbositybehaviour.h"
using namespace std;

class ChaseSubState : public SoccerState
{
public:
    ChaseSubState(ChaseState* parent) : SoccerState(parent), m_parent_machine(parent) {};
    virtual ~ChaseSubState() {};
protected:
    ChaseState* m_parent_machine;

};

class GoToBall : public ChaseSubState
{
public:
    GoToBall(ChaseState* parent) : ChaseSubState(parent) 
    {
        m_pan_end_time = 0;
        m_pan_started = false;
        m_previous_time = 0;
        m_has_fallen = false;

        head_behaviour = HeadBehaviour::getInstance();
    }
    ~GoToBall() {}
protected:
    HeadBehaviour* head_behaviour;
    BehaviourState* nextState()
    {   // do state transitions in the chase state machine
        return this;
    }
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "GoToBall" << endl;
        #endif
        if (m_parent->stateChanged() or m_data->CurrentTime - m_previous_time > 200)
        {
            m_pan_end_time = 0;
            m_pan_started = false;
            m_pan_finished = false;
        }
        
        Self& self = Blackboard->Objects->self;
        MobileObject& ball = Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL];
        bool iskicking, iswalking = false;
        m_data->get(NUSensorsData::MotionKickActive, iskicking);
        //m_data->get(NUSensorsData::MotionWalkActive, iswalking);
        iskicking = iskicking and not iswalking;
        
        // if (pan not run)
        //      if (pan start condition)
        //          start pan
        //      else if (its been a long time since a pan)
        //          set pan as not having run
        
        // if (not panning) then
        //      do usual track ball
        
        /*if (m_data->CurrentTime >= m_pan_end_time+200) {
            Blackboard->lookForBall = true;
        } else if (m_data->CurrentTime <= m_pan_end_time and not m_pan_finished) {
            Blackboard->lookForBall = false;
        }*/
        
        
        float targetKickDistance = 13.f;
        float balldistance = ball.estimatedDistance();
        float ballbearing = ball.estimatedBearing();
        
        if (ball.isObjectVisible()) {
                ballbearing = ball.measuredBearing();
                balldistance = ball.measuredDistance();//*cos(ball.measuredElevation();
        }
        vector<float> speed;
        if(not iskicking)
        {
            /*Attempt to fix kick (jake):
              swapped from goToBallDirectWithSidewardsKick to goToBall*/
            speed = BehaviourPotentials::goToBall(ball, self, BehaviourPotentials::getBearingToOpponentGoal(m_field_objects, m_game_info),targetKickDistance-4.f,42.);
           // speed = BehaviourPotentials::goToBallDirectWithSidewardsKick(ball, self, BehaviourPotentials::getBearingToOpponentGoal(m_field_objects, m_game_info),targetKickDistance,37.);
            vector<float> result;

            // if the ball is too far away to kick and the obstable is closer than the ball we need to dodge!
            result = speed;
            
            //if ((m_pan_started and not m_pan_finished or not m_pan_started) and ball.estimatedDistance() < targetKickDistance and BehaviourPotentials::opponentsGoalLinedUp(m_field_objects, m_game_info))
            //    result = vector<float>(3,0);
            
            m_jobs->addMotionJob(new WalkJob(result[0], result[1], result[2]));
            #if DEBUG_BEHAVIOUR_VERBOSITY > 2
                debug << m_data->CurrentTime << ": Going to Ball - (" << result[0] << ", " << result[1] << ", " << result[2] << ")" << endl;
            #endif
        }
        
        
        if((balldistance < targetKickDistance) and fabs(ballbearing) < 1.1 and not ball.lost())//&& 
            //( BehaviourPotentials::opponentsGoalLinedUp(m_field_objects, m_game_info) ))// && //XXX: fix goal lineup to use side kicks too
              //fabs(ball.estimatedBearing()) > 0.25 && //ball is not "between" our feet
              //fabs(ball.estimatedBearing()) < 0.75) //ball is not "outside" our feet
              // && ball.TimeSeen() > 0 && m_pan_finished)
        {
            //m_jobs->addMotionJob(new WalkJob(0, 0, 0));
            vector<float> kickPosition(2,0);
            vector<float> targetPosition(2,0);
            vector<float> goalPosition(3,0);
            goalPosition = self.CalculateDifferenceFromGoal(BehaviourPotentials::getOpponentGoal(m_field_objects, m_game_info));
            kickPosition[0] = balldistance * cos(ballbearing);
            kickPosition[1] = balldistance * sin(ballbearing);
            targetPosition[0] = goalPosition[0] * cos(goalPosition[1]);
            targetPosition[1] = goalPosition[0] * sin(goalPosition[1]);
            KickJob* kjob = new KickJob(0,kickPosition, targetPosition);
            m_jobs->addMotionJob(kjob);
            //cout << "Kick! " << ballbearing << "; " << goalPosition[1] << endl;
           // cout << "GoalRel: " << targetPosition[0] << ", " << targetPosition[1] << endl;
            #if DEBUG_BEHAVIOUR_VERBOSITY > 2
                debug << m_data->CurrentTime << ": Kicking Ball at distance " << balldistance << endl;
            #endif
        } 
        //Get head behaviour decision
        head_behaviour->makeVisionChoice(HeadBehaviour::CheckAgentPolicy);//or pass HeadBehaviour::(M)RLAgentPolicy for learning to be active via either motivated or world rewards

    }





    
private:
    float m_pan_end_time;
    bool m_pan_started, m_pan_time_captured, m_pan_finished;
    float m_previous_time;
    bool m_has_fallen;
};

class FindTarget : public ChaseSubState
{
public:
    FindTarget(ChaseState* parent) : ChaseSubState(parent) {}
    ~FindTarget() {};
protected:
    BehaviourState* nextState()
    {   // do state transitions in the chase state machine
        return this;
    }
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "FindTarget" << endl;
        #endif
    }
};

class Kick : public ChaseSubState
{
public:
    Kick(ChaseState* parent) : ChaseSubState(parent) {}
    ~Kick() {};
    BehaviourState* nextState()
    {   // do state transitions in the chase state machine
        return this;
    }
protected:
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "Kick" << endl;
        #endif
    }
};


#endif


