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
public:
//classification so we know which foot to line the ball up with
    enum DIRECTION 
        {
        LEFT = 1,
        BOTH = 0,
        RIGHT = -1
        };
    enum WALK_TYPE 
        {
        USELASTCOMMAND = -1,
        GOTOPOINT = 0,
        GOTOOBJECT = 1,
        GOTOBALL = 2
        };

private:
    
    

//XXX: load all these from central config ----------------------------------------
    //load these values from walk config
    static constexpr float m_turn_speed = 0.4;
    static constexpr float m_walk_speed = 0.95;
    static constexpr float m_feet_separation = 14.0;
    
    //load from robot model
    static constexpr float m_foot_size = 10.0;
    
    //timers for starting turning and walking
    static constexpr double m_walk_start_time = 0.2;
    static constexpr double m_walk_turn_time = 0.2;
    
    //walk accel/deccel controls
    static constexpr double m_acceleration_time = 0.2;
    static constexpr float m_acceleration_fraction = 0.5;
    
    //approach speeds
    static constexpr float m_close_approach_speed = 0.8;
    static constexpr float m_close_approach_distance = 30.0;
    static constexpr float m_mid_approach_speed = 0.85;
    static constexpr float m_mid_approach_distance = 60.0;
    
    //turning values
    static constexpr float m_turn_deviation = 0.1;
    
    //hystereses
    static constexpr float m_distance_hysteresis = 10.0;
    static constexpr float m_turning_hysteresis = 0.1;
    static constexpr float m_position_hysteresis = 30.0;
    
    //ball lineup
    std::vector<float> m_ball_approach_angle;
    std::vector<int> m_ball_kick_foot;
    static constexpr float m_ball_lineup_distance = 10.0;
    static constexpr int m_ball_lineup_min_distance = 6.0;
    
    //extra config options
    static constexpr bool m_use_localisation_avoidance = false;
    static constexpr float m_assumed_obstacle_width = 25.0;
    static constexpr float m_avoid_distance = 50.0;
//END config variables section-------------------------------------------------------------------
    
    //hysteresis variables
    int m_turning;
    int m_distance_increment;
    int m_approach_distance;
    
    //info for the current walk
    Object* current_object;
    std::vector<float> current_point;
    int current_command;
    std::vector<float> current_walk_command;
    std::vector<float> m_raw_move;
    float current_heading;

    bool kick_;

    Navigation() : kick_(false) {}
    
    /*! @brief Given a distance, and relative bearing and heading, returns a new walk command based on the navigation parameters
     */
    std::vector<float> generateWalk(float distance, float relative_bearing, float relative_heading, bool avoidObstacles = true);
    
    /*! @brief Returns a new direction (bearing) to move that avoids all obstacles
     */
    float avoidObstacles(const std::vector<float> position, float distance, float relative_bearing);
    
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
    
    int getCurrentCommand() {
        return current_command;
    }
    
    bool isStopped() {
        const float epsilon = 0.05;
        if (current_walk_command.size() > 0) {
            return current_walk_command[0] < epsilon and mathGeneral::abs(current_walk_command[1]) < epsilon and mathGeneral::abs(current_walk_command[2]) < epsilon;
        } else {
            return false;
        }
    }
        
    /*! @brief Go to a point and face a heading. Returned std::vector is walk command std::vector.
     */
    std::vector<float> goToPoint(float distance, float relative_bearing, float relative_heading);
    
    /*! @brief Go to a point and face a heading. Returned std::vector is walk command std::vector.
     */
    std::vector<float> goToPoint(Object* fieldObject, float heading);
    
    /*! @brief Go to a point and face a heading. Returned std::vector is walk command std::vector.
     */
    std::vector<float> goToPoint(const std::vector<float> point);
    
    /*! @brief Approach the ball with a good angle to kick from. Returned std::vector is walk command std::vector. Buggy.
     */
    std::vector<float> goToBall(Object* kickTarget = NULL);
    
    /*! @brief Approach the ball with a good angle to kick from. Returned std::vector is walk command std::vector. Fixed version.
     */
    std::vector<float> goToBall2(Object* kickTarget = NULL);

    /*! @brief Stop the walk.
     */
    std::vector<float> stop();

    /*! @brief A wrapper to send kick commands. The kick should decide whether to listen or not.
     */
    void kick();
    
    /*! @brief Update the goto calculations and send the walk commands (if actions are not active).
     */
    void update();
    
    static Navigation* getInstance();
    
};

#endif

