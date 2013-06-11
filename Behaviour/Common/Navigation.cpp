/*! @file Navigation.cpp
    @brief centrally defined navigation commands for new behaviours.

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

#include "Navigation.h"
#include "Tools/Math/General.h"
#include "Behaviour/Common/NavigationLogic.h"
#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"

vector<float> Navigation::generateWalk(float distance, float relative_bearing, float relative_heading, bool avoidObstacles) {
    vector<float> new_walk(3,0);
    double current_time;
    float walk_speed;
    float walk_bearing;
    
    if (avoidObstacles) {
        relative_bearing = this->avoidObstacles(NavigationLogic::getSelfPosition(),distance, relative_bearing);
    }
    
    //check what distance increment we're in:
    if (distance > m_mid_approach_distance+m_distance_hysteresis) {
        m_distance_increment = 3;
        walk_speed = 1.f;
    } else if (distance > m_close_approach_distance + m_distance_hysteresis and
               distance < m_mid_approach_distance) {
        m_distance_increment = 2;
        walk_speed = m_mid_approach_speed;
    } else if (distance > m_ball_lineup_distance + m_distance_hysteresis and
               distance < m_close_approach_distance) {
        m_distance_increment = 1;
        walk_speed = m_close_approach_speed;
    } else {
        m_distance_increment = 0;
        walk_speed = 0.f;
    }
    
    //decide between heading and bearing
    if (m_distance_increment > 0) {
        walk_bearing = relative_bearing;
    } else {
        walk_bearing = relative_heading;
    }
    
    //check turning hysteresis
    if (m_turning < 0 and walk_bearing < -m_turn_deviation) {
        walk_speed = min(walk_bearing,m_turn_speed);
    } else if (m_turning > 0 and walk_bearing > m_turn_deviation) {
        walk_speed = min(walk_bearing,m_turn_speed);
    } else {
        walk_bearing = 0;
    }
    
    new_walk[0] = walk_speed;
    new_walk[2] = walk_bearing;
    return new_walk;
}

float Navigation::avoidObstacles(const vector<float> position, float distance, float relative_bearing) {
    float new_bearing = relative_bearing;
    float avoid_distance = min(m_avoid_distance,distance);
    vector<Object> obstacles;
    
    
    
    //use either localised or visual avoidance
    if (m_use_localisation_avoidance) {
        //XXX: localisation based avoidance not implemented
        //wait for localisation to track obstacles
        
    } else {
        obstacles = NavigationLogic::getVisibleObstacles();
        for(unsigned int i=0; i < obstacles.size(); i++) { //for each object
            if (obstacles[i].measuredDistance() < avoid_distance) { //if we are an obstacle
                if (obstacles[i].measuredBearing() > relative_bearing and obstacles[i].measuredBearing()-obstacles[i].arc_width < relative_bearing) { 
                    //if we are on the right and occluding
                    new_bearing = mathGeneral::normaliseAngle(obstacles[i].measuredBearing()-obstacles[i].arc_width);
                } else if (obstacles[i].measuredBearing() < relative_bearing and obstacles[i].measuredBearing()+obstacles[i].arc_width > relative_bearing) { 
                    //if we are on the left and occluding
                    new_bearing = mathGeneral::normaliseAngle(obstacles[i].measuredBearing()+obstacles[i].arc_width);
                }
            }
        }
    }
    
    return new_bearing;
}
    

float Navigation::alignFoot(float distance, float relative_bearing, int use_foot) {
    float new_bearing = relative_bearing;
    
    //XXX: do foot choosing
    
    return new_bearing;
}
    

void Navigation::updateConfiguration() {
    //XXX: set variables
}

void Navigation::resetHystereses() {
    //set turning to 0 (neither left nor right)
    m_turning = 0;
    
    //set distance increment to 3 (max)
    m_distance_increment = 3;
}
    

vector<float> Navigation::goToPoint(float distance, float relative_bearing, float relative_heading) {
    
    
    
    //set continuing movement policy
    current_command = USELASTCOMMAND;
    
    //must generate the walk last
    current_walk_command = generateWalk(distance,relative_bearing,relative_heading);
    return current_walk_command;
}
    

vector<float> Navigation::goToPoint(Object* fieldObject, float heading) {
    
    //calculate the desired move
    vector<float> self = NavigationLogic::getSelfPosition();
    vector<float> destination = NavigationLogic::getObjectPosition(*fieldObject);
    destination[2] = heading;
    vector<float> move = NavigationLogic::getPositionDifference(self,destination);
    
    //set continuing movement policy
    //XXX: be very careful - can only do this with objects we don't delete
    current_object = fieldObject;
    current_heading = heading;
    if (current_command != GOTOOBJECT)
        resetHystereses();
    current_command = GOTOOBJECT;
    
    //must generate the walk last
    current_walk_command = generateWalk(move[0],move[1],move[2]);
    return current_walk_command;
}
    

vector<float> Navigation::goToPoint(const vector<float> point) {
    
    //calculate the desired move
    vector<float> self = NavigationLogic::getSelfPosition();
    vector<float> move = NavigationLogic::getPositionDifference(self,point);
    
    
    //set continuing movement policy
    current_point = point;
    if (current_command != GOTOPOINT)
        resetHystereses();
    current_command = GOTOPOINT;
    
    //must generate the walk last
    current_walk_command = generateWalk(move[0],move[1],move[2]);
    return current_walk_command;
}


vector<float> Navigation::goToBall(Object* kickTarget) {
    
    //calculate the desired move
    vector<float> self = NavigationLogic::getSelfPosition();
    vector<float> point = NavigationLogic::getBallPosition();
    vector<float> move = NavigationLogic::getPositionDifference(self,point);
    
    //set continuing movement policy
    //XXX: this only works if the kick target isn't deleted ever
    current_object = kickTarget;
    if (current_command != GOTOBALL)
        resetHystereses();
    current_command = GOTOBALL;
    
    //XXX: fix ball approach
    
    //must generate the walk last
    current_walk_command = generateWalk(move[0],move[1],move[2]);
    return current_walk_command;
}
    

void Navigation::update() {
    
    //update our calculations
    switch (current_command) 
        {
        case USELASTCOMMAND:
            break;
        case GOTOPOINT:
            goToPoint(current_point);
            break;
        case GOTOOBJECT:
            goToPoint(current_object,current_heading);
            break;
        case GOTOBALL:
            goToBall(current_object);
            break;
        }
    
    //set the walkjob
    Blackboard->Jobs->addMotionJob(new WalkJob(current_walk_command[0], current_walk_command[1], current_walk_command[2]));
}

Navigation* Navigation::getInstance() {
    //if (!Instance) {
    static Navigation* Instance = new Navigation();
    //}
    return Instance;
}

vector<float> Navigation::stop() {
    
    
    
    //set continuing movement policy
    current_command = USELASTCOMMAND;
    
    //must generate the walk last
    current_walk_command = generateWalk(0,0,0);
    return current_walk_command;
}

void Navigation::kick() {
    //set the kick
    Blackboard->Jobs->addMotionJob(new KickJob(Blackboard->Sensors->GetTimestamp(),NavigationLogic::getBallPosition(), NavigationLogic::getOpponentGoalPosition()));
}


