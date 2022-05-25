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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "HeadBehaviourSoccer.hpp"

#include <string>
#include <utility>

#include "extension/Configuration.hpp"

#include "message/localisation/Field.hpp"
#include "message/motion/GetupCommand.hpp"
#include "message/motion/HeadCommand.hpp"
#include "message/vision/Ball.hpp"
#include "message/vision/Goal.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/coordinates.hpp"
#include "utility/motion/InverseKinematics.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"


namespace module::behaviour::skills {

    using extension::Configuration;
    using message::input::Sensors;
    using message::motion::ExecuteGetup;
    using message::motion::HeadCommand;
    using message::motion::KillGetup;
    using message::vision::Ball;
    using VisionBalls = message::vision::Balls;

    using utility::input::ServoID;
    using utility::math::coordinates::sphericalToCartesian;
    using utility::math::geometry::Quad;
    using utility::motion::kinematics::calculateHeadJoints;
    using utility::support::Expression;

    inline Eigen::Vector2d screenAngularFromObjectDirection(const Eigen::Vector3d& v) {
        // Returns {yaw, pitch}
        return {std::atan2(v.y(), v.x()), std::atan2(v.z(), v.x())};
    }

    HeadBehaviourSoccer::HeadBehaviourSoccer(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("HeadBehaviourSoccer.yaml")
            .then("Head Behaviour Soccer Config", [this](const Configuration& config) {
                log_level         = config["log_level"].as<NUClear::LogLevel>();
                search_timeout_ms = config["search_timeout_ms"].as<float>();
                fixation_time_ms  = config["fixation_time_ms"].as<float>();
                max_yaw           = config["max_yaw"].as<float>();
                min_yaw           = config["min_yaw"].as<float>();
                max_pitch         = config["max_pitch"].as<float>();
                min_pitch         = config["min_pitch"].as<float>();

                // Create vector of search positions
                search_positions.push_back(Eigen::Vector2d(min_yaw, max_pitch));
                search_positions.push_back(Eigen::Vector2d(min_yaw, min_pitch));
                search_positions.push_back(Eigen::Vector2d(0, min_pitch));
                search_positions.push_back(Eigen::Vector2d(max_yaw, min_pitch));
                search_positions.push_back(Eigen::Vector2d(max_yaw, max_pitch));
                search_positions.push_back(Eigen::Vector2d(0, max_pitch));
            });


        // TODO(BehaviourTeam): remove this horrible code
        // Check to see if we are currently in the process of getting up.
        on<Trigger<ExecuteGetup>>().then([this] { isGettingUp = true; });

        // Check to see if we have finished getting up.
        on<Trigger<KillGetup>>().then([this] { isGettingUp = false; });

        // Updates the last seen time of ball
        on<Trigger<VisionBalls>, With<Sensors>>().then([this](const VisionBalls& balls, const Sensors& sensors) {
            if (!balls.balls.empty()) {
                log<NUClear::DEBUG>("Ball Seen trigger");
                ballLastMeasured = NUClear::clock::now();
                rBCc = Eigen::Vector3d(sphericalToCartesian(balls.balls[0].measurements[0].srBCc.cast<double>()));
            }
        });

        on<Trigger<Sensors>, Single, Sync<HeadBehaviourSoccer>>().then(
            "Head Behaviour Main Loop",
            [this](const Sensors& sensors) {
                // Get the time since the ball was last seen
                float timeSinceBallLastMeasured =
                    std::chrono::duration_cast<std::chrono::duration<float>>(NUClear::clock::now() - ballLastMeasured)
                        .count();
                // log<NUClear::DEBUG>("Time since seen ball: ", timeSinceBallLastMeasured);
                // Only look for ball if not getting up
                if (!isGettingUp) {
                    if (timeSinceBallLastMeasured < search_timeout_ms / 1000) {
                        // We can see the ball, lets look at it
                        // Apply a simple exponential filter to the ball position to smooth out the noisy spikes in ball
                        // position
                        Eigen::Vector2d angles = screenAngularFromObjectDirection(rBCc);
                        log<NUClear::DEBUG>("Ball (x,y,z): (", rBCc[0], ",", rBCc[1], ",", rBCc[2], ")");
                        log<NUClear::DEBUG>("Angle to ball (yaw, pitch): (", angles[0], ",", angles[1], ")");
                        std::unique_ptr<HeadCommand> command = std::make_unique<HeadCommand>();
                        command->yaw                         = angles[0];
                        command->pitch                       = angles[1];
                        command->robot_space                 = true;
                        command->smooth                      = true;
                        emit(std::move(command));
                    }
                    else {
                        // We haven't seen the ball in a while, lets look around
                        // log<NUClear::DEBUG>("We haven't seen the ball in a while, lets look around");
                        float timeSinceLastSearchMoved = std::chrono::duration_cast<std::chrono::duration<float>>(
                                                             NUClear::clock::now() - searchLastMoved)
                                                             .count();
                        // log<NUClear::DEBUG>("Time since last moved position: ", timeSinceLastSearchMoved);

                        if (timeSinceLastSearchMoved > fixation_time_ms / 1000) {
                            searchLastMoved                      = NUClear::clock::now();
                            std::unique_ptr<HeadCommand> command = std::make_unique<HeadCommand>();
                            command->yaw                         = search_positions[searchIdx][0];
                            command->pitch                       = search_positions[searchIdx][1];
                            command->robot_space                 = true;
                            command->smooth                      = false;
                            searchIdx++;
                            emit(std::move(command));
                        }

                        // Reset the search position index if at end of list
                        if (searchIdx == search_positions.size()) {
                            searchIdx = 0;
                        }
                    }
                }
            });
    }

}  // namespace module::behaviour::skills
