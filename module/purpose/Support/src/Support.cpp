/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "Support.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/purpose/Player.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/support/GlobalConfig.hpp"

#include "utility/math/euler.hpp"

namespace module::purpose {

    using extension::Configuration;

    using SupportMsg = message::purpose::Support;

    using message::input::GameState;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::strategy::WalkToFieldPosition;
    using message::support::FieldDescription;
    using message::support::GlobalConfig;

    using utility::math::euler::pos_rpy_to_transform;

    Support::Support(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Support.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Support.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Configuration, With<FieldDescription>>("Formation.yaml")
            .then([this](const Configuration& config, const FieldDescription& fd) {
                // Resolves a YAML coord node: scalar, { field, scale }, or { position[, offset] }
                auto resolve = [&fd](const YAML::Node& n) -> double {
                    if (n.IsScalar()) return n.as<double>();
                    if (n["field"]) {
                        std::string dim = n["field"].as<std::string>();
                        return (dim == "length" ? fd.dimensions.field_length : fd.dimensions.field_width)
                               * n["scale"].as<double>();
                    }
                    std::string pos = n["position"].as<std::string>();
                    double off      = n["offset"] ? n["offset"].as<double>() : 0.0;
                    if (pos == "field_min_x") return -fd.dimensions.field_length / 2.0 + off;
                    if (pos == "left_goal_area_max_x")
                        return -fd.dimensions.field_length / 2.0 + fd.dimensions.goal_area_length + off;
                    if (pos == "goal_area_min_y") return -fd.dimensions.goal_area_width / 2.0 + off;
                    if (pos == "goal_area_max_y") return fd.dimensions.goal_area_width / 2.0 + off;
                    return off;
                };

                // Read top-level defaults (used when mode or robot doesn't specify their own)
                Eigen::Vector2d default_attraction{config["defaults"]["attraction"]["x"].as<double>(),
                                                   config["defaults"]["attraction"]["y"].as<double>()};
                double default_min_x = resolve(config["defaults"]["minX"]);

                cfg.modes.clear();

                for (auto mode : config["modes"]) {
                    std::string mode_name = mode.first.as<std::string>();

                    // Some modes define their own defaults that override the top-level ones
                    Eigen::Vector2d mode_attraction =
                        mode.second["defaults"]["attraction"]
                            ? Eigen::Vector2d{mode.second["defaults"]["attraction"]["x"].as<double>(),
                                              mode.second["defaults"]["attraction"]["y"].as<double>()}
                            : default_attraction;
                    double mode_min_x = mode.second["defaults"]["minX"]
                                            ? resolve(mode.second["defaults"]["minX"])
                                            : default_min_x;

                    for (auto robot : mode.second["robots"]) {
                        int id  = std::stoi(robot.first.as<std::string>());
                        auto& n = robot.second;

                        // Robot-level values override mode defaults if present
                        RobotSlot slot;
                        slot.offset     = {resolve(n["offset"]["x"]), resolve(n["offset"]["y"])};
                        slot.attraction = n["attraction"] ? Eigen::Vector2d{n["attraction"]["x"].as<double>(),
                                                                            n["attraction"]["y"].as<double>()}
                                                          : mode_attraction;
                        slot.min_x      = n["minX"] ? resolve(n["minX"]) : mode_min_x;

                        cfg.modes[mode_name][id] = slot;
                    }
                }
            });

        on<Provide<SupportMsg>,
           Optional<With<Ball>>,
           With<Sensors>,
           With<Field>,
           With<GameState>,
           With<GlobalConfig>,
           With<FieldDescription>>()
            .then([this](const std::shared_ptr<const Ball>& ball,
                         const Sensors& sensors,
                         const Field& field,
                         const GameState& game_state,
                         const GlobalConfig& global_config,
                         const FieldDescription& fd) {
                // Select mode
                std::string mode_name = "normal_play";
                if (game_state.mode.value == GameState::Mode::THROW_IN)
                    mode_name = game_state.our_kick_off ? "throw_in_us" : "throw_in_them";
                else if (game_state.our_kick_off)
                    mode_name = "kickoff_us";
                else if (!game_state.our_kick_off)
                    mode_name = "kickoff_them";

                // Look up this robot's slot
                auto mode_it = cfg.modes.find(mode_name);
                if (mode_it == cfg.modes.end())
                    mode_it = cfg.modes.find("normal_play");
                auto& robots = mode_it->second;
                auto slot_it = robots.find(global_config.player_id);
                if (slot_it == robots.end())
                    return;  // no slot for this robot
                const auto& slot = slot_it->second;

                // Calculate target position
                Eigen::Vector3d position{slot.offset.x(), slot.offset.y(), 0};
                if (ball) {
                    Eigen::Vector3d rBFf = field.Hfw * ball->rBWw;
                    // Formation.yaml comes from another team and uses a field x-axis convention that is
                    // mirrored relative to ours, so express the ball's x in that same convention before
                    // combining it with the formation coefficients (which are given in that convention)
                    double ball_x = -rBFf.x();
                    position.x()  = std::max(slot.min_x, slot.offset.x() + slot.attraction.x() * ball_x);
                    position.y()  = slot.offset.y() + slot.attraction.y() * rBFf.y();
                }

                // Clamp to field
                double half_length = fd.dimensions.field_length / 2.0;
                double half_width  = fd.dimensions.field_width / 2.0;
                position.x()       = std::clamp(position.x(), -half_length, half_length);
                position.y()       = std::clamp(position.y(), -half_width, half_width);

                // Convert out of Formation.yaml's mirrored x convention and into our own field frame
                position.x() = -position.x();

                // Flip yaw axis to work with other teams formation rules
                emit<Task>(
                    std::make_unique<WalkToFieldPosition>(pos_rpy_to_transform(position, Eigen::Vector3d(0, 0, M_PI)),
                                                          true));
            });
    }

}  // namespace module::purpose
