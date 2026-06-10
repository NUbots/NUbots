/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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
#include "ReadyAttack.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/purpose/Player.hpp"
#include "message/strategy/FindBall.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/math/euler.hpp"

namespace module::purpose {

    using extension::Configuration;

    using message::input::GameState;
    using message::localisation::Ball;
    using message::localisation::Field;
    using ReadyAttackTask = message::purpose::ReadyAttack;
    using message::strategy::FindBall;
    using message::strategy::PositionBehindBall;
    using message::strategy::WalkToFieldPosition;
    using message::support::FieldDescription;

    using utility::math::euler::pos_rpy_to_transform;

    ReadyAttack::ReadyAttack(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("ReadyAttack.yaml").then([this](const Configuration& config) {
            // Use configuration here from file ReadyAttack.yaml
            this->log_level             = config["log_level"].as<NUClear::LogLevel>();
            cfg.center_circle_offset    = config["center_circle_offset"].as<double>();
            cfg.penalty_defend_distance = config["penalty_defend_distance"].as<double>();
        });

        on<Provide<ReadyAttackTask>, With<GameState>, With<FieldDescription>, With<Field>, With<Ball>>().then(
            [this](const GameState& game_state, const FieldDescription& fd, const Field& field, const Ball& ball) {
                // Ready state may happen during penalty positioning or kick off
                // Kickoff will happen in normal mode
                if (game_state.mode == GameState::Mode::NORMAL) {
                    // Waiting for kick off, position outside the center circle
                    log<DEBUG>("Waiting for kick off...");
                    Eigen::Vector3d rPFf =
                        Eigen::Vector3d(fd.dimensions.center_circle_diameter / 2 + cfg.center_circle_offset, 0.0, 0.0);
                    emit<Task>(std::make_unique<WalkToFieldPosition>(
                        utility::math::euler::pos_rpy_to_transform(rPFf, Eigen::Vector3d(0, 0, -M_PI)),
                        true));
                    return;
                }

                // If we are not waiting for kick off, it is the penalty positioning phase
                // Determine if we are the attacker or not
                bool attacker = game_state.secondary_state.team_performing == game_state.team.team_id;

                // If we are defending, position between the ball and our goal at the distance specified in the rules
                if (!attacker) {
                    // Position of the center of the goals in field coordinates
                    log<DEBUG>("Defending penalty, positioning...");
                    Eigen::Vector3d rGFf =
                        Eigen::Vector3d((fd.dimensions.field_length / 2) + fd.dimensions.goal_depth, 0.0, 0.0);
                    // Position of the ball in field coordinates
                    Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;

                    // Unit vector from goal to ball
                    Eigen::Vector3d uGBf = (rGFf - rBFf).normalized();
                    // Move the ball position by the penalty defend distance
                    Eigen::Vector3d rPFf = rBFf + (uGBf * cfg.penalty_defend_distance);

                    // Rotation should face the ball, get the angle from the field to the ball
                    double angle = std::atan2(rBFf.y(), rBFf.x());

                    // Emit a task to walk to the position, facing the ball
                    emit<Task>(std::make_unique<WalkToFieldPosition>(
                        utility::math::euler::pos_rpy_to_transform(rPFf, Eigen::Vector3d(0, 0, angle)),
                        true));
                }

                // We are attacking, and should position to take the ball towards the opponent's goal
                log<DEBUG>("Attacking penalty, positioning...");
                emit<Task>(std::make_unique<PositionBehindBall>());
            });
    }

}  // namespace module::purpose
