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
#include "Attack.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/planning/KickTo.hpp"
#include "message/purpose/Player.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/Who.hpp"
#include "message/support/FieldDescription.hpp"

namespace module::purpose {

    using extension::Configuration;

    using AttackMsg = message::purpose::Attack;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::planning::KickTo;
    using message::strategy::TackleBall;
    using message::strategy::WalkToKickBall;
    using message::strategy::Who;
    using message::support::FieldDescription;

    Attack::Attack(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Attack.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Attack.yaml
            this->log_level        = config["log_level"].as<NUClear::LogLevel>();
            cfg.kick_when          = config["kick_when"].as<std::string>();
            cfg.possession_timeout = duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["possession_timeout"].as<double>()));
        });

        on<Provide<AttackMsg>, With<Ball>, With<Field>, With<FieldDescription>>().then(
            [this](const AttackMsg& attack, const Ball& ball, const Field& field, const FieldDescription& fd) {
                // Always request a kick task
                if (cfg.kick_when == "Always") {
                    emit<Task>(std::make_unique<KickTo>(), 1);
                }
                // Only kick in the attacking third
                else if (cfg.kick_when == "AttackingThird") {
                    double attacking_third = -fd.dimensions.field_length / 3;
                    Eigen::Vector3d rBFf   = field.Hfw * ball.rBWw;
                    // If the ball is in the attacking third of the field, activate the kick
                    if (rBFf.x() < attacking_third) {
                        log<DEBUG>("Ball in attacking third, kick!");
                        emit<Task>(std::make_unique<KickTo>(), 1);  // kick the ball towards the goal
                    }
                }
                // If kick_when is never, do not request the kick task

                // Check if the ball is in our possession
                bool in_possession_proposal = !(attack.ball_pos == message::strategy::Who::OPPONENT);
                // Confirm possession
                confirm_possession(in_possession_proposal);

                if (in_possession) {
                    log<DEBUG>("We have the ball or it is free, walk to the goal!");
                    emit<Task>(std::make_unique<TackleBall>(), 0);
                    emit<Task>(std::make_unique<WalkToKickBall>(), 1);
                }
                else {
                    // If the opponent has had the ball for longer than the timeout, we assume they are in possession
                    log<DEBUG>("Opponent has the ball, tackle it!");
                    emit<Task>(std::make_unique<TackleBall>(), 1);
                    emit<Task>(std::make_unique<WalkToKickBall>(), 0);
                }
            });
    }

}  // namespace module::purpose
