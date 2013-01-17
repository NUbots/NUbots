/*! @file GoalKeeperProvider.h
    @brief Declaration of a behaviour provider for testing the goal keeper behaviour
 
    @class GoalKeeperProvider
    @brief A special behaviour for developing the goal keeper
 

    @author Jason Kulk
 
  Copyright (c) 2011 Jason Kulk
 
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

#ifndef NUWALKTEST_PROVIDER_H
#define NUWALKTEST_PROVIDER_H

#include "../BehaviourProvider.h"
#include "Motion/Tools/MotionScript.h"

class NUWalkTestProvider : public BehaviourProvider
{
public:
    NUWalkTestProvider (Behaviour* manager);
    ~NUWalkTestProvider();
protected:
    void doBehaviour();
    void goToPoint(float x, float y, float h);
    void stop();
    void testWalkVector(float magnitude,float direction, float turn_speed, double run_time);

private:
    //possible states
    enum WALK_STATE {
        STOP = 0,
        GOTOPOINT = 1,
        TESTDIRECTION = 2
        };
        
    
    //walk engine movement vector
    float m_trans_speed,m_trans_direction,m_turn_speed;
    
    //position when current command started
    float m_start_x,m_start_y,m_start_h;
    float m_end_x,m_end_y,m_end_h;
    
    //odometry counters
    float m_odom_x,m_odom_y,m_odom_h;
    
    //goal position
    float m_goal_x,m_goal_y,m_goal_h;
    
    //state tracking variable
    int m_current_state,m_last_state;
    
    //timers for state change and performance measurement
    double m_command_start,m_command_end,m_run_time;
    
    bool m_has_fallen;
};


#endif

