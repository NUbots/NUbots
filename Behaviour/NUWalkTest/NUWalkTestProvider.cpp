/*! @file SlipTestProvider.cpp
    @brief Implementation of slip testing behaviour class

    @author Josiah Walker
 
 Copyright (c) 2012 Josiah Walker
 
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

#include "NUWalkTestProvider.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/GameInformation/GameInformation.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/MotionFreezeJob.h"

#include "Infrastructure/FieldObjects/FieldObjects.h"

#include "Behaviour/BehaviourPotentials.h"

#include <math.h>
#include <numeric>
#include "debug.h"
#include "debugverbositybehaviour.h"
#include "Tools/Math/StlVector.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Math/General.h"

using namespace std;

NUWalkTestProvider::NUWalkTestProvider(Behaviour* manager) : BehaviourProvider(manager)
{
    Self& self = m_field_objects->self;
    m_start_x = self.wmX();
    m_start_y = self.wmY();
    m_start_h = self.Heading();
    
    m_command_start = Blackboard->Sensors->CurrentTime;
    
    m_current_state = STOP;
    m_last_state = m_current_state;
}

void NUWalkTestProvider::goToPoint(float x, float y, float h) {
    Self& self = m_field_objects->self;
    m_start_x = self.wmX();
    m_start_y = self.wmY();
    m_start_h = self.Heading();
    m_has_fallen = false;
    
    m_goal_x = x;
    m_goal_y = y;
    m_goal_h = h;
    
    m_command_start = Blackboard->Sensors->CurrentTime;
    
    m_current_state = GOTOPOINT;
}

void NUWalkTestProvider::stop() {
    Self& self = m_field_objects->self;
    m_end_x = self.wmX();
    m_end_y = self.wmY();
    m_end_h = self.Heading();
    
    vector<float> odometry;
    Blackboard->Sensors->getOdometry(odometry);
    m_odom_x = odometry[0];
    m_odom_y = odometry[1];
    m_odom_h = odometry[2];
    
    m_command_end = Blackboard->Sensors->CurrentTime;
    m_last_state = m_current_state;
    m_current_state = STOP;
}

void NUWalkTestProvider::testWalkVector(float magnitude,float direction, float turn_speed, double run_time) {
    Self& self = m_field_objects->self;
    m_start_x = self.wmX();
    m_start_y = self.wmY();
    m_start_h = self.Heading();
    
    m_trans_speed = magnitude;
    m_trans_direction = direction;
    m_turn_speed = turn_speed;
    m_run_time = run_time;
    m_has_fallen = false;
    
    m_command_start = Blackboard->Sensors->CurrentTime;
    
    m_current_state = TESTDIRECTION;
}


NUWalkTestProvider::~NUWalkTestProvider()
{
}

void NUWalkTestProvider::doBehaviour()
{
    // hack it, and put the GameState into Playing (this turns on localisation)
    while (m_game_info->getCurrentState() != GameInformation::PlayingState)
        m_game_info->doManualStateChange();
    
    Self& self = m_field_objects->self;
    float xd,yd,hd;
    
    //execute the actions
    switch (m_current_state) {
        case STOP:
            Blackboard->Jobs->addMotionJob(new WalkJob(0.,0.,0.));
            break;
        case GOTOPOINT:
            xd = self.wmX()-m_goal_x;
            yd = self.wmY()-m_goal_y;
            hd = self.Heading()-m_goal_h;
            if (xd*xd+yd*yd >= 225. and hd*hd >= 0.6) { // arbitrarily chosen end accuracy - just to get the robot ot an approx position
                vector<float> fieldstate;
                fieldstate.push_back(m_goal_x);
                fieldstate.push_back(m_goal_h);
                fieldstate.push_back(m_goal_y);
                vector<float> walkdirection = BehaviourPotentials::goToFieldState(self,fieldstate);
                Blackboard->Jobs->addMotionJob(new WalkJob(walkdirection[0]/1.2,walkdirection[1],walkdirection[2]/2.));
            } else {
                stop();
            }
            m_has_fallen = m_has_fallen or Blackboard->Sensors->isFallen();
            break;
        case TESTDIRECTION:
            if (m_command_start + m_run_time >= Blackboard->Sensors->CurrentTime) {
                Blackboard->Jobs->addMotionJob(new WalkJob(m_trans_speed,m_trans_direction,m_turn_speed));
            } else {
                stop();
            }
            m_has_fallen = m_has_fallen or Blackboard->Sensors->isFallen();
            break;
    }
    
    //add all your logic in here - robot will return to stop after the timeout for testing
    //and after reaching the goal point for gotopoint. Check m_last_state for last action.
    if (m_current_state == STOP and Blackboard->Sensors->CurrentTime > m_command_end + 100.) {
        switch (m_last_state) {
            case STOP:
                //program has just started up
                break;
            case GOTOPOINT:
                //we have just gone back to the centre point
                break;
            case TESTDIRECTION:
                //we have just tested a walk command
                break;
        }
    }
    
}



