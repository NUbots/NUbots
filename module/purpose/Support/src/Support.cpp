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
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/purpose/Player.hpp"
#include "message/purpose/SupportPosition.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/support/GlobalConfig.hpp"

#include "utility/math/euler.hpp"

namespace module::purpose {

    using extension::Configuration;

    using SupportMsg = message::purpose::Support;

    using message::input::GameState;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::purpose::SupportPosition;
    using message::strategy::LookAtBall;
    using message::strategy::WalkToFieldPosition;
    using message::support::FieldDescription;
    using message::support::GlobalConfig;

    using utility::math::euler::pos_rpy_to_transform;

    Support::Support(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Support.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Support.yaml
            this->log_level       = config["log_level"].as<NUClear::LogLevel>();
            cfg.stop_threshold    = config["stop_threshold"].as<double>();
            cfg.stopped_threshold = config["stopped_threshold"].as<double>();
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
                    if (pos == "field_max_x") return fd.dimensions.field_length / 2.0 + off;
                    if (pos == "field_min_y") return -fd.dimensions.field_width / 2.0 + off;
                    if (pos == "field_max_y") return fd.dimensions.field_width / 2.0 + off;
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
                double default_max_x = resolve(config["defaults"]["maxX"]);
                double default_min_y = resolve(config["defaults"]["minY"]);
                double default_max_y = resolve(config["defaults"]["maxY"]);

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
                    double mode_max_x = mode.second["defaults"]["maxX"]
                                            ? resolve(mode.second["defaults"]["maxX"])
                                            : default_max_x;
                    double mode_min_y = mode.second["defaults"]["minY"]
                                            ? resolve(mode.second["defaults"]["minY"])
                                            : default_min_y;
                    double mode_max_y = mode.second["defaults"]["maxY"]
                                            ? resolve(mode.second["defaults"]["maxY"])
                                            : default_max_y;

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
                        slot.max_x      = n["maxX"] ? resolve(n["maxX"]) : mode_max_x;
                        slot.min_y      = n["minY"] ? resolve(n["minY"]) : mode_min_y;
                        slot.max_y      = n["maxY"] ? resolve(n["maxY"]) : mode_max_y;

                        cfg.modes[mode_name][id] = slot;
                    }
                }
            });

        on<Provide<SupportMsg>,
           Optional<With<Ball>>,
           With<Field>,
           With<GameState>,
           With<GlobalConfig>,
           With<FieldDescription>>()
            .then([this](const std::shared_ptr<const Ball>& ball,
                         const Field& field,
                         const GameState& game_state,
                         const GlobalConfig& global_config,
                         const FieldDescription& fd) {
                auto position = calculate_support_position(ball, field, game_state, global_config, fd);
                if (!position)
                    return;  // no slot for this robot (or config not yet loaded)

                // Make robot always face ball while playing
                double yaw = M_PI;
                if (ball && game_state.phase.value == GameState::Phase::PLAYING) {
                    Eigen::Vector3d rBFf = field.Hfw * ball->rBWw;
                    yaw                  = std::atan2(rBFf.y() - position->y(), rBFf.x() - position->x());
                }

                emit<Task>(std::make_unique<WalkToFieldPosition>(
                    pos_rpy_to_transform(*position, Eigen::Vector3d(0, 0, yaw)),
                    true,
                    cfg.stop_threshold,
                    cfg.stopped_threshold));

                // Always track the ball with the head, regardless of game phase.
                emit<Task>(std::make_unique<LookAtBall>());
            });

        // Continuously compute and emit what this robot's support position would be right now, even when
        // it isn't currently assigned the Support purpose - lets NUsight show a live "what if" preview
        // regardless of which robot is actually supporting.
        on<Every<10, Per<std::chrono::seconds>>,
           Optional<With<Ball>>,
           With<Field>,
           With<GameState>,
           With<GlobalConfig>,
           With<FieldDescription>>()
            .then([this](const std::shared_ptr<const Ball>& ball,
                         const Field& field,
                         const GameState& game_state,
                         const GlobalConfig& global_config,
                         const FieldDescription& fd) {
                auto position = calculate_support_position(ball, field, game_state, global_config, fd);
                if (!position)
                    return;  // no slot for this robot (or config not yet loaded)

                auto support_position       = std::make_unique<SupportPosition>();
                support_position->player_id = global_config.player_id;
                support_position->position  = position->head<2>();
                emit(support_position);
            });
    }

    std::optional<Eigen::Vector3d> Support::calculate_support_position(const std::shared_ptr<const Ball>& ball,
                                                                        const Field& field,
                                                                        const GameState& game_state,
                                                                        const GlobalConfig& global_config,
                                                                        const FieldDescription& fd) const {
        // Select the formation mode matching the current set play, with the kicking team
        // (our_kick_off tracks the GameController's kicking_team) picking the us/them variant
        const std::string suffix = game_state.our_kick_off ? "_us" : "_them";
        std::string mode_name;
        switch (game_state.mode.value) {
            case GameState::Mode::DIRECT_FREEKICK: mode_name = "direct_free_kick" + suffix; break;
            case GameState::Mode::INDIRECT_FREEKICK: mode_name = "indirect_free_kick" + suffix; break;
            case GameState::Mode::PENALTYKICK: mode_name = "penalty_kick" + suffix; break;
            case GameState::Mode::CORNER_KICK: mode_name = "corner_kick" + suffix; break;
            case GameState::Mode::GOAL_KICK: mode_name = "goal_kick" + suffix; break;
            case GameState::Mode::THROW_IN: mode_name = "throw_in" + suffix; break;
            default:
                // No set play: kickoff formation while positioning in ready, normal play otherwise
                mode_name = game_state.phase.value == GameState::Phase::READY ? "kickoff" + suffix : "normal_play";
                break;
        }

        // Look up this robot's slot; bail out if the formation config cannot supply one
        const RobotSlot* slot_ptr = find_slot(mode_name, global_config.player_id);
        if (slot_ptr == nullptr)
            return std::nullopt;  // no slot for this robot (or config not yet loaded)
        const auto& slot = *slot_ptr;

        // Calculate target position
        Eigen::Vector3d position{slot.offset.x(), slot.offset.y(), 0};
        if (ball) {
            Eigen::Vector3d rBFf = field.Hfw * ball->rBWw;
            // Formation.yaml uses a mirrored x-axis, so flip the ball's x to match before combining it
            // with the formation coefficients
            double ball_x = -rBFf.x();
            position.x() =
                std::clamp(slot.offset.x() + slot.attraction.x() * ball_x, slot.min_x, slot.max_x);
            position.y() =
                std::clamp(slot.offset.y() + slot.attraction.y() * rBFf.y(), slot.min_y, slot.max_y);
        }

        // Clamp to field
        double half_length = fd.dimensions.field_length / 2.0;
        double half_width  = fd.dimensions.field_width / 2.0;
        position.x()       = std::clamp(position.x(), -half_length, half_length);
        position.y()       = std::clamp(position.y(), -half_width, half_width);

        // Convert out of Formation.yaml's mirrored x convention and into our own field frame
        position.x() = -position.x();

        return position;
    }

    auto Support::find_slot(const std::string& mode_name, int player_id) const -> const RobotSlot* {
        // Select the requested formation mode, falling back to normal_play when it is not defined
        auto mode_it = cfg.modes.find(mode_name);
        if (mode_it == cfg.modes.end())
            mode_it = cfg.modes.find("normal_play");

        // Neither the mode nor the normal_play fallback exists (e.g. Formation.yaml not loaded yet)
        if (mode_it == cfg.modes.end())
            return nullptr;

        // Look up this robot's slot within the selected mode
        const auto& robots = mode_it->second;
        auto slot_it       = robots.find(player_id);
        return slot_it == robots.end() ? nullptr : &slot_it->second;
    }

}  // namespace module::purpose
