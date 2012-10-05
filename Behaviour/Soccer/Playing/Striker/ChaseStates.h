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
    }
    ~GoToBall() {};
protected:
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
            //vector<float> speed = BehaviourPotentials::goToBall(ball, self, BehaviourPotentials::getBearingToOpponentGoal(m_field_objects, m_game_info),targetKickDistance-4.f,42.);
            speed = BehaviourPotentials::goToBallDirectWithSidewardsKick(ball, self, BehaviourPotentials::getBearingToOpponentGoal(m_field_objects, m_game_info),targetKickDistance,37.);
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
            //cout << "GoalRel: " << targetPosition[0] << ", " << targetPosition[1] << endl;
            #if DEBUG_BEHAVIOUR_VERBOSITY > 2
                debug << m_data->CurrentTime << ": Kicking Ball at distance " << balldistance << endl;
            #endif
        } 
        
        
        /*NEW VISION
        //-----------------------------------------------------------------------------------------------*/
        HeadBehaviour* head_behaviour = HeadBehaviour::getInstance();
        head_behaviour->makeVisionChoice(HeadBehaviour::TimeVSCostPriority);

        
        /*OLD VISION
        //-----------------------------------------------------------------------------------------------
        static bool getting_up = false;
        static double getup_timer = 0.f;
        bool amichanging = getting_up;
        m_data->get(NUSensorsData::MotionGetupActive, getting_up);
        if (getting_up == false and amichanging == true) {
            getup_timer = m_data->CurrentTime;
        }
        
        if (not m_pan_started and not iskicking and fabs(speed[2]) > 0.01 or m_has_fallen and not Blackboard->Sensors->isFallen() and getup_timer-m_data->CurrentTime > 500.)
        {   
            
            if ((m_data->CurrentTime - m_pan_end_time > 5000) and
                //fabs(BehaviourPotentials::getBearingToOpponentGoal(m_field_objects, m_game_info)) < 1.3 and ball.TimeSeen() > 1000 and
                (not ball.estimatedDistance() < 20.f or m_has_fallen))
            {   
                //Blackboard->lookForBall = false;
                StationaryObject& yellow_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST];
                StationaryObject& yellow_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST];
                StationaryObject& blue_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST];
                StationaryObject& blue_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST];
                
                float timesinceyellowgoalseen = min(yellow_left.TimeSinceLastSeen(), yellow_right.TimeSinceLastSeen());
                float timesincebluegoalseen = min(blue_left.TimeSinceLastSeen(), blue_right.TimeSinceLastSeen());
                float timesincegoalseen = min(timesinceyellowgoalseen, timesincebluegoalseen);
                float hackfactor = ball.estimatedDistance()/20.; //(9.0/35000.0)*timesincegoalseen + 1;
                
                float bearing_to_yellow = self.CalculateBearingToStationaryObject(yellow_left);
                float bearing_to_blue = self.CalculateBearingToStationaryObject(blue_right);
                
                vector<StationaryObject> posts;
                if (fabs(bearing_to_yellow) < fabs(bearing_to_blue))
                {
                    posts.push_back(yellow_left);
                    posts.push_back(yellow_right);
                }
                else
                {
                    posts.push_back(blue_left);
                    posts.push_back(blue_right);
                }
                if (not m_has_fallen) {
                    m_jobs->addMotionJob(new HeadPanJob(posts, hackfactor));
                } else {
                    m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation));
                }
                m_pan_started = true;
                m_pan_end_time = m_data->CurrentTime + 800;
                m_pan_time_captured = false;
                m_pan_finished = false;
                m_has_fallen = false;
                #if DEBUG_BEHAVIOUR_VERBOSITY > 2
                    debug << m_data->CurrentTime << ": Goal Post Pan Started" << endl;
                #endif
                //cout << m_data->CurrentTime << ": Goal Post Pan Started" << endl;
            }
        }
        else if (m_pan_finished and m_data->CurrentTime - m_pan_end_time > 2900 or ball.estimatedDistance() < 20.f)
        {
            m_pan_started = false;
            m_pan_finished = false;
        }
        
        if (not m_pan_started and Blackboard->Sensors->isFallen()) {
            m_has_fallen = true;
        }
        
        // this is a hack to get the pan end time right given the delay in the update of the motion sensors
        // above we set the pan time to be 1s ahead, and then only update the end time when its longer
        // ie. after it has been updated.
        if (m_pan_started and not m_pan_finished)
        {
            float endtime;
            m_data->get(NUSensorsData::MotionHeadCompletionTime, endtime);
            if (not m_pan_time_captured and endtime > m_pan_end_time)
            {
                m_pan_end_time = endtime;
                m_pan_time_captured = true;
            }
            if (m_data->CurrentTime >= m_pan_end_time)
                m_pan_finished = true;
        }
        
        if (not m_pan_started or m_pan_finished)
        {
            
            if (ball.TimeSinceLastSeen() > 1800)
            {
                //cout << m_data->CurrentTime << ": Ball Pan" << endl;
                m_jobs->addMotionJob(new HeadPanJob(ball, 0.5));
            }
            else
            {
                //cout << m_data->CurrentTime << ": Ball Pan" << endl;
                if (ball.isObjectVisible())
                {
                    #if DEBUG_BEHAVIOUR_VERBOSITY > 2
                        debug << m_data->CurrentTime << ": Tracking ball" << endl;
                    #endif
                    //cout << m_data->CurrentTime << ": Tracking ball" << endl;
                    m_jobs->addMotionJob(new HeadTrackJob(ball));
                }
                else if (ball.TimeSinceLastSeen() < 70)
                {
                    m_jobs->addMotionJob(new HeadPanJob(ball, 0.1));
                }
            }
        }
        
        // this is a HUGE hack
        // I am feeling really lazy at the moment; I am updating the time the ball was last seen even though it is NOT seen
        // This is to simply prevent the ball from being lost during the look away pan (this will also trick the other robots via a team information)
        if (m_pan_started and not m_pan_finished)       
            ball.updateTimeLastSeen(m_data->CurrentTime - 750);
        
        
        //cout << balldistance << "\t" << ballbearing << endl;
        
        #if DEBUG_BEHAVIOUR_VERBOSITY > 2
            debug << m_data->CurrentTime << ": pan_started: " << m_pan_started << " pan_finished: " << m_pan_finished << " pan end time: " << m_pan_end_time << endl; 
            debug << m_data->CurrentTime << ": ball distance: " << ball.estimatedDistance() << " ball seen: " << ball.TimeSeen() << endl; 
            debug << " kicking: " << iskicking << endl;
        #endif
        m_previous_time = m_data->CurrentTime;

        */

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


