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
#include "Defend.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/purpose/Player.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/math/euler.hpp"

namespace module::purpose {

    using extension::Configuration;

    using DefendMsg = message::purpose::Defend;

    using message::localisation::Ball;
    using message::localisation::Field;
    using message::strategy::WalkToFieldPosition;
    using message::support::FieldDescription;
    using utility::math::euler::pos_rpy_to_transform;

    Defend::Defend(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Defend.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Defend.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<DefendMsg>, With<Ball>, With<Field>, With<FieldDescription>>().then(
            [this](const Ball& ball, const Field& field, const FieldDescription& fd) {
                // In this state, we are expected to hang back in the case that we become the attacking robot
                // Stay in the line of sight between the ball and the goal

                // Vector from the field to the goal
                Eigen::Vector3d rGFf(fd.dimensions.field_length / 2, 0, 0);

                // The penalty line across the width of the field
                double penalty_line_x = fd.dimensions.field_length / 2 - fd.dimensions.penalty_area_length;

                // Ball position in field frame
                Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;

                // Ball to goal unit vector
                Eigen::Vector3d uGBf = (rGFf - rBFf).normalized();

                // Angle from goal to ball in field frame
                double goal_ball_angle = std::atan2(-uGBf.y(), -uGBf.x());

                // Position is the intersection of the penalty line and the line from the ball to the goal
                double y_position = rBFf.y() + ((uGBf.y() / uGBf.x()) * (penalty_line_x - rBFf.x()));

                // Walk to the optimal position
                emit<Task>(std::make_unique<WalkToFieldPosition>(
                    pos_rpy_to_transform(Eigen::Vector3d(penalty_line_x, y_position, 0.0),
                                         Eigen::Vector3d(0, 0, goal_ball_angle)),
                    true));
            });
    }


}  // namespace module::purpose
