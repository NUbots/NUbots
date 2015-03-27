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
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/angle.h"

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

        double robot_heading_ = 0;
        arma::vec2 odom_old_robot_position_ = { 0, 0 };
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

      

    
        utility::math::matrix::Transform2D robotPose;
        utility::math::matrix::Transform2D ballPose;
        utility::math::matrix::Transform2D ball_position_;
        utility::math::matrix::Transform2D ball_velocity_;
        utility::math::matrix::Transform2D robot_position_;
        utility::math::matrix::Transform2D robot_velocity_;
        utility::math::matrix::Transform2D robot_odometry_;

    public:
        /// @brief Called by the powerplant to build and setup the SoccerSimulator reactor.
        explicit SoccerSimulator(std::unique_ptr<NUClear::Environment> environment);
    };

}
}


#endif