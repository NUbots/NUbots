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
#include "messages/support/GlobalConfig.h"
#include "messages/support/FieldDescription.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/angle.h"
#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/input/Sensors.h"
#include "messages/input/CameraParameters.h"
#include "messages/motion/KickCommand.h"

#include "VirtualVision.h"

namespace modules {
namespace support {

    struct SoccerSimulatorConfig {
        static constexpr const char* CONFIGURATION_PATH = "SoccerSimulator.yaml";
    };

    class SoccerSimulator : public NUClear::Reactor {
    public:
        enum MotionType{
            NONE = 0,
            PATH = 1,
            MOTION = 2
        };
        MotionType motionTypeFromString(std::string s){
            if(s.compare("NONE")==0){
                return MotionType::NONE;
            } else
            if(s.compare("PATH")==0){
                return MotionType::PATH;
            } else
            if(s.compare("MOTION")==0){
                return MotionType::MOTION;
            } else {
                return MotionType::PATH;
            }
        }

        enum PathType{
            SIN = 0,
            TRIANGLE = 1
        };

        PathType pathTypeFromString(std::string s){
            if(s.compare("SIN")==0){
                return PathType::SIN;
            } else
            if(s.compare("TRIANGLE")==0){
                return PathType::TRIANGLE;
            } else {
                return PathType::SIN;
            }
        }

    private:
        time_t moduleStartupTime;
        double absolute_time();

        //Member variables
        messages::motion::KickPlannerConfig kick_cfg;

        std::shared_ptr<messages::support::FieldDescription> field_description_;

        static constexpr size_t SIMULATION_UPDATE_FREQUENCY = 180;

        struct Config{

            bool simulate_goal_observations = true;
            bool simulate_ball_observations = true;
            bool distinguish_left_and_right_goals = true;

            struct Motion{
                MotionType motion_type = MotionType::PATH;
                struct Path{
                    float period = 10;
                    PathType type = PathType::SIN;
                    float x_amp = 3;
                    float y_amp = 2;
                } path;
            };

            Motion robot;
            Motion ball;

            bool emit_robot_fieldobjects = true;
            bool emit_ball_fieldobjects = true;

            bool blind_robot = false;
            bool auto_start_behaviour = true;

            arma::vec4 vision_error = {0.01,0.017,0.017};

        } cfg_;

        //Goal models
        std::vector<VirtualGoalPost> goalPosts;

        //World State
        struct WorldState {
            //Transform2D == (x,y,heading)
            utility::math::matrix::Transform2D robotPose;
            utility::math::matrix::Transform2D robotVelocity;
            // utility::math::matrix::Transform2D ballPose;
            // utility::math::matrix::Transform2D ballVelocity;
            VirtualBall ball;
        };

        WorldState world;

        std::queue<messages::motion::KickCommand> kickQueue;



        bool kicking;
        uint PLAYER_ID;

        //Methods
        void updateConfiguration(const messages::support::Configuration<SoccerSimulatorConfig>& config, const messages::support::GlobalConfig& globalConfig);

        std::unique_ptr<messages::platform::darwin::DarwinSensors::Gyroscope> computeGyro(float heading, float oldHeading);

        arma::vec2 getPath(Config::Motion::Path p);

    public:
        /// @brief Called by the powerplant to build and setup the SoccerSimulator reactor.
        explicit SoccerSimulator(std::unique_ptr<NUClear::Environment> environment);
    };

}
}


#endif
