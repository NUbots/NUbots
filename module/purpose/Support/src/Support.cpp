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
#include "Support.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/purpose/Player.hpp"
#include "message/strategy/FindBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/math/euler.hpp"

namespace module::purpose {

    using extension::Configuration;

    using SupportMsg = message::purpose::Support;

    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::strategy::FindBall;
    using message::strategy::WalkToFieldPosition;
    using message::support::FieldDescription;

    using utility::math::euler::pos_rpy_to_transform;

    Support::Support(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Support.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Support.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<SupportMsg>, With<Ball>, With<Sensors>, With<Field>, With<FieldDescription>>().then(
            [this](const Ball& ball, const Sensors& sensors, const Field& field, const FieldDescription& fd) {
                // Find the ball if we don't have it
                emit<Task>(std::make_unique<FindBall>(), 1);  // Need to know where the ball is

                // Get ball in field coordinates
                Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;
                Eigen::Vector3d rRFf = (field.Hfw * sensors.Hrw.inverse()).translation();

                // Keep in line with the ball on the x-axis, but stay on the other side of the field on the y-axis
                Eigen::Vector3d position = rBFf;

                // Choose the side closest to our current position
                double left_side_y  = rBFf.y() - (fd.dimensions.field_width / 4);
                double right_side_y = rBFf.y() + (fd.dimensions.field_width / 4);

                // Check which side we're closer to
                bool prefer_left = std::abs(rRFf.y() - left_side_y) < std::abs(rRFf.y() - right_side_y);
                position.y()     = prefer_left ? left_side_y : right_side_y;

                // If there isn't really space to walk to the preferred side, walk to the other side
                if (position.y() > fd.dimensions.field_width / 2) {
                    position.y() = rBFf.y() - (fd.dimensions.field_width / 4);
                }
                else if (position.y() < -fd.dimensions.field_width / 2) {
                    position.y() = rBFf.y() + (fd.dimensions.field_width / 4);
                }

                // Bound x by the penalty areas
                double penalty_area = fd.dimensions.field_length / 2 - fd.dimensions.penalty_area_length;
                position.x()        = std::clamp(position.x(), -penalty_area, penalty_area);
                // Closest side, unless it wont fit
                // Walk to the position
                emit<Task>(
                    std::make_unique<WalkToFieldPosition>(pos_rpy_to_transform(position, Eigen::Vector3d(0, 0, M_PI)),
                                                          true));
            });
    }

}  // namespace module::purpose
