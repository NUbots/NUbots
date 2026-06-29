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

#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/purpose/Player.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/math/euler.hpp"

namespace module::purpose {

    using extension::Configuration;

    using SupportMsg = message::purpose::Support;

    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::strategy::WalkToFieldPosition;
    using message::support::FieldDescription;

    using utility::math::euler::pos_rpy_to_transform;

    Support::Support(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Support.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Support.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

        });

        on<Configuration>("formation.yaml").then([this](const Configuration& config) {
            // Use configuration here from file formation.yaml
            // Read top-level defaults (used when mode or robot doesn't specify their own)
            Eigen::Vector2d default_attraction{config["defaults"]["attraction"]["x"].as<double>(),
                                            config["defaults"]["attraction"]["y"].as<double>()};
            double default_min_x = config["defaults"]["minX"].as<double>();

            for (auto mode : config["modes"]) {
                std::string mode_name = mode.first.as<std::string>();

                // Some modes define their own defaults that override the top-level ones
                Eigen::Vector2d mode_attraction = mode.second["defaults"]["attraction"]
                    ? Eigen::Vector2d{mode.second["defaults"]["attraction"]["x"].as<double>(),
                                    mode.second["defaults"]["attraction"]["y"].as<double>()}
                    : default_attraction;
                double mode_min_x = mode.second["defaults"]["minX"]
                    ? mode.second["defaults"]["minX"].as<double>()
                    : default_min_x;

                for (auto robot : mode.second["robots"]) {
                    int id  = std::stoi(robot.first.as<std::string>());
                    auto& n = robot.second;

                    // Robot-level values override mode defaults if present
                    RobotSlot slot;
                    slot.offset     = {n["offset"]["x"].as<double>(), n["offset"]["y"].as<double>()};
                    slot.attraction = n["attraction"]
                        ? Eigen::Vector2d{n["attraction"]["x"].as<double>(), n["attraction"]["y"].as<double>()}
                        : mode_attraction;
                    slot.min_x = n["minX"] ? n["minX"].as<double>() : mode_min_x;

                    cfg.modes[mode_name][id] = slot;
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
                else if (!game_state.our_kick_off && /* secondary time still running */ ...)
                    mode_name = "kickoff_them";

                // Look up this robot's slot
                auto mode_it = cfg.modes.find(mode_name);
                if (mode_it == cfg.modes.end()) mode_it = cfg.modes.find("normal_play");
                auto& robots = mode_it->second.robots;
                auto slot_it = robots.find(global_config.player_id);
                if (slot_it == robots.end()) return;  // no slot for this robot
                const auto& slot = slot_it->second;

                // Calculate target position
                Eigen::Vector3d position{slot.offset.x(), slot.offset.y(), 0};
                if (ball) {
                    Eigen::Vector3d rBFf = field.Hfw * ball->rBWw;
                    position.x() = std::max(slot.min_x, slot.offset.x() + slot.attraction.x() * rBFf.x());
                    position.y() = slot.offset.y() + slot.attraction.y() * rBFf.y();
                }

                // Clamp to field
                double half_length = fd.dimensions.field_length / 2.0;
                double half_width  = fd.dimensions.field_width / 2.0;
                position.x() = std::clamp(position.x(), -half_length, half_length);
                position.y() = std::clamp(position.y(), -half_width, half_width);

                emit<Task>(std::make_unique<WalkToFieldPosition>(
                    pos_rpy_to_transform(position, Eigen::Vector3d(0, 0, 0)), true));
            });
    }

}  // namespace module::purpose
