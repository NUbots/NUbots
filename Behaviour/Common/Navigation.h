/*! @file Navigation.h
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



#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/FieldObjects/Object.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Tools/Math/General.h"



class Navigation {
private:
    //classification so we know which foot to line the ball up with
    enum DIRECTION 
        {
        LEFT = 1,
        BOTH = 0,
        RIGHT = -1;
        };
    enum WALK_TYPE 
        {
        USELASTCOMMAND = -1,
        GOTOPOINT = 0,
        GOTOOBJECT = 1,
        GOTOBALL = 2
        };
    

//XXX: load all these from central config ----------------------------------------
    //load these values from walk config
    float m_turn_speed;
    float m_walk_speed;
    float m_feet_separation;
    
    //load from robot model
    float m_foot_size;
    
    //timers for starting turning and walking
    double m_walk_start_time;
    double m_walk_turn_time;
    
    //walk accel/deccel controls
    double m_acceleration_time;
    float m_acceleration_fraction;
    
    //approach speeds
    float m_close_approach_speed;
    float m_close_approach_distance;
    float m_mid_approach_speed;
    float m_mid_approach_distance;
    
    //turning values
    float m_turn_deviation;
    float m_turn_speed;
    
    //state controls
    int m_turning;
    int m_approach_distance;
    
    //hystereses
    float m_distance_hysteresis;
    float m_turning_hysteresis;
    float m_position_hysteresis;
    
    //ball lineup
    std::vector<float> m_ball_approach_angle;
    std::vector<int> m_ball_kick_foot;
    float m_ball_lineup_distance;
    int m_ball_lineup_min_distance;
    
    //extra config options
    bool m_use_localisation_avoidance;
    float m_assumed_obstacle_width;
    float m_avoid_distance;
//END config variables section-------------------------------------------------------------------
    
    //hysteresis variables
    int m_turning;
    int m_distance_increment;
    
    //info for the current walk
    Object* current_object;
    std::vector<float> current_point;
    int current_command;
    std::vector<float> current_walk_command;
    float current_heading;
    
    /*! @brief Given a distance, and relative bearing and heading, returns a new walk command based on the navigation parameters
     */
    std::vector<float> generateWalk(float distance, float relative_bearing, float relative_heading, bool avoidObstacles = true);
    
    /*! @brief Returns a new direction (bearing) to move that avoids all obstacles
     */
    float avoidObstacles(const std::vector<float> position, float relative_bearing);
    
    /*! @brief Returns a new direction (bearing) to move that aligns the designated foot with the ball
     */
    float alignFoot(float distance, float relative_bearing, int use_foot);
    
    /*! @brief Updates the configuration values of the Navigation module.
     */
    void updateConfiguration();
    
    /*! @brief reset hystereses on walk command type change.
     */
    void resetHystereses();
    
public:
    
        
    /*! @brief Go to a point and face a heading. Returned std::vector is walk command std::vector.
     */
    std::vector<float> goToPoint(float distance, float relative_bearing, float relative_heading);
    
    /*! @brief Go to a point and face a heading. Returned std::vector is walk command std::vector.
     */
    std::vector<float> goToPoint(Object fieldObject, float heading);
    
    /*! @brief Go to a point and face a heading. Returned std::vector is walk command std::vector.
     */
    std::vector<float> goToPoint(const std::vector<float> point);
    
    /*! @brief Approach the ball with a good angle to kick from. Returned std::vector is walk command std::vector.
     */
    std::vector<float> goToBall(Object kickTarget = NULL);
    
    /*! @brief Update the goto calculations and send the walk commands (if actions are not active).
     */
    void update();
    
};

#endif

