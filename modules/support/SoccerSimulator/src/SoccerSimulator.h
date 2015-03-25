/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_LOCALISATION_SOCCERSIMULATOR_H
#define MODULES_LOCALISATION_SOCCERSIMULATOR_H

#include <nuclear>
#include <armadillo>
#include "messages/support/Configuration.h"
#include "messages/support/FieldDescription.h"

namespace modules {
namespace support {

    struct SoccerSimulatorConfig {
        static constexpr const char* CONFIGURATION_PATH = "SoccerSimulatorConfig.yaml";
    };

    class SoccerSimulator : public NUClear::Reactor {
    public:
        enum MotionType{
            NONE = 0,
            PATH = 1,
            MOTION = 2
        };
        MotionType motionTypeFromString(std::string s){
            if(s.compare("NONE")){
                return MotionType::NONE;
            } else
            if(s.compare("PATH")){
                return MotionType::PATH;
            } else
            if(s.compare("MOTION")){
                return MotionType::MOTION;
            } else {
                return MotionType::PATH;
            }
        }

        enum PathType{
            SIN = 0,
            TRIANGLE = 1
        };

        MotionType motionTypeFromString(std::string s){
            if(s.compare("SIN")){
                return MotionType::SIN;
            } else
            if(s.compare("TRIANGLE")){
                return MotionType::TRIANGLE;
            } else {
                return MotionType::SIN;
            }
        }

    private:
        void UpdateConfiguration(
            const messages::support::Configuration<SoccerSimulatorConfig>& config);

        std::unique_ptr<messages::platform::darwin::DarwinSensors::Gyroscope> computeGyro(float dHeading);

        arma::vec2 ball_position_ = { 0, 0 };
        arma::vec2 ball_velocity_ = { 0, 0 };
        arma::vec2 robot_position_ = { 0, 0 };
        arma::vec2 robot_velocity_ = { 0, 0 };
        arma::vec2 world_imu_direction_ = { 0, 1 };
        arma::vec3 robot_imu_direction_ = { 0, 1, 0 };
        arma::vec2 robot_odometry_ = { 0, 0 };
        // arma::vec robot_heading_ = { 1, 0 };
        // double robot_heading_ = 3.141;
        double robot_heading_ = 0;
        arma::vec2 odom_old_robot_position_ = { 0, 0 };
        // arma::vec odom_old_robot_heading_ = { 1, 0 };
        double odom_old_robot_heading_ = 0;

        std::shared_ptr<messages::support::FieldDescription> field_description_;

        struct {
            bool simulate_goal_observations = true;
            bool simulate_ball_observations = true;
            bool observe_left_goal = true;
            bool observe_right_goal = true;
            bool distinguish_left_and_right_goals = true;

            struct MotionConfig{
                MotionType motion_type = MotionType::PATH;
                struct {
                    float period = 1000;
                    float PathType = PathType::SIN;
                } path;
            };

            MotionConfig robot;
            MotionConfig ball;

            bool emit_robot_fieldobjects = true;
            bool emit_ball_fieldobjects = true;

        } cfg_;

        struct FieldPose{
            arma::vec2 p = arma::vec2({0,0});
            float heading = 0;
        };

        FieldPose robotPose;
        FieldPose ballPose;

    public:
        /// @brief Called by the powerplant to build and setup the SoccerSimulator reactor.
        explicit SoccerSimulator(std::unique_ptr<NUClear::Environment> environment);
    };

}
}


#endif