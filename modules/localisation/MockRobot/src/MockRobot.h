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

#ifndef MODULES_LOCALISATION_MOCKROBOT_H
#define MODULES_LOCALISATION_MOCKROBOT_H

#include <nuclear>
#include <armadillo>
#include "messages/support/Configuration.h"
#include "messages/support/FieldDescription.h"

namespace modules {
namespace localisation {

    struct MockRobotConfig {
        static constexpr const char* CONFIGURATION_PATH = "MockRobotConfig.yaml";
    };

    class MockRobot : public NUClear::Reactor {
    private:
        void UpdateConfiguration(
            const messages::support::Configuration<MockRobotConfig>& config);

        arma::vec ball_position_ = { 0, 0 };
        arma::vec ball_velocity_ = { 0, 0 };
        arma::vec robot_position_ = { 0, 0 };
        arma::vec robot_velocity_ = { 0, 0 };
        // arma::vec robot_heading_ = { 1, 0 };
        double robot_heading_ = 0;
        arma::vec odom_old_robot_position_ = { 0, 0 };
        // arma::vec odom_old_robot_heading_ = { 1, 0 };
        double odom_old_robot_heading_ = 0;

        std::shared_ptr<messages::support::FieldDescription> field_description_;

        struct {
            bool simulate_vision;
            bool simulate_goal_observations;
            bool simulate_ball_observations;
            bool simulate_odometry;
            bool simulate_robot_movement;
            bool simulate_ball_movement;
            bool emit_robot_fieldobjects;
            bool emit_ball_fieldobjects;
        } cfg_;

    public:
        /// @brief Called by the powerplant to build and setup the MockRobot reactor.
        explicit MockRobot(std::unique_ptr<NUClear::Environment> environment);
    };

}
}


#endif