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

#include <utility>

#include "extension/Configuration.hpp"

#include "message/motion/GetupCommand.hpp"
#include "message/motion/HeadCommand.hpp"
#include "message/vision/Ball.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/support/yaml_expression.hpp"


namespace module::behaviour::skills {

    using extension::Configuration;

    using message::motion::ExecuteGetup;
    using message::motion::HeadCommand;
    using message::motion::KillGetup;
    using VisionBalls = message::vision::Balls;

    using utility::math::coordinates::reciprocalSphericalToCartesian;
    using utility::math::coordinates::sphericalToCartesian;
    using utility::support::Expression;

    /**
     * Calculates angles to a 3d vector in world
     *
     * @return Eigen::Vector2d of {yaw, pitch}
     */
    inline Eigen::Vector2d screenAngularFromObjectDirection(const Eigen::Vector3d& v) {
        return {std::atan2(v.y(), v.x()), std::atan2(v.z(), v.x())};
    }

    HeadBehaviourSoccer::HeadBehaviourSoccer(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("HeadBehaviourSoccer.yaml")
            .then("Head Behaviour Soccer Config", [this](const Configuration& config) {
                log_level         = config["log_level"].as<NUClear::LogLevel>();
                search_timeout_ms = config["search_timeout_ms"].as<float>();
                fixation_time_ms  = config["fixation_time_ms"].as<float>();

                // Create vector of search positions
                for (const auto& position : config["positions"].config) {
                    search_positions.push_back(position.as<Expression>());
                }
            });


        // TODO(BehaviourTeam): remove this horrible code
        // Check to see if we are currently in the process of getting up.
        on<Trigger<ExecuteGetup>>().then([this] { isGettingUp = true; });

        // Check to see if we have finished getting up.
        on<Trigger<KillGetup>>().then([this] { isGettingUp = false; });

        // Updates the last seen time of ball
        on<Trigger<VisionBalls>>().then([this](const VisionBalls& balls) {
            if (!balls.balls.empty()) {
                ballLastMeasured = NUClear::clock::now();
                rBCc             = reciprocalSphericalToCartesian(balls.balls[0].measurements[0].srBCc.cast<double>());
            }
        });

        on<Every<90, Per<std::chrono::seconds>>, Sync<HeadBehaviourSoccer>>().then(
            "Head Behaviour Main Loop",
            [this]() {
                // Get the time since the ball was last seen
                float timeSinceBallLastMeasured =
                    std::chrono::duration_cast<std::chrono::duration<float>>(NUClear::clock::now() - ballLastMeasured)
                        .count();
                // Only look for ball if not getting up
                if (!isGettingUp) {
                    if (timeSinceBallLastMeasured < search_timeout_ms / 1000) {
                        // We can see the ball, lets look at it
                        Eigen::Vector2d angles               = screenAngularFromObjectDirection(rBCc);
                        std::unique_ptr<HeadCommand> command = std::make_unique<HeadCommand>();
                        command->yaw                         = angles[0];
                        command->pitch                       = angles[1];
                        command->robot_space                 = true;
                        command->smooth                      = true;
                        emit(std::move(command));
                    }
                    else {
                        // Ball hasn't been seen in a while. Look around using search positions
                        float timeSinceLastSearchMoved = std::chrono::duration_cast<std::chrono::duration<float>>(
                                                             NUClear::clock::now() - searchLastMoved)
                                                             .count();

                        // Robot will move through the search positions, and linger for fixation_time_ms. Once
                        // fixation_time_ms time has passed, send a new head command for the next position in the list
                        // of search_positions
                        if (timeSinceLastSearchMoved > fixation_time_ms / 1000) {
                            // Move to next search position in list
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
