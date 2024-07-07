/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#include "WalkInsideBoundedBox.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/strategy/WalkInsideBoundedBox.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

#include "utility/math/euler.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::strategy {

    using extension::Configuration;
    using WalkInsideBoundedBoxTask = message::strategy::WalkInsideBoundedBox;
    using utility::support::Expression;
    using Ball = message::localisation::Ball;
    using message::localisation::Field;
    using message::strategy::WalkToFieldPosition;

    using utility::math::euler::pos_rpy_to_transform;

    WalkInsideBoundedBox::WalkInsideBoundedBox(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {
        on<Configuration>("WalkInBoundedBox.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });
        on<Provide<WalkInBoundedBoxTask>, Trigger<Ball>, With<Field>>().then(
            [this](const WalkInBoundedBoxTask& box, const Ball& ball, const Field& field) {
                // Get the current position of the ball on the field
                Eigen::Isometry3d Hfw = field.Hfw;
                Eigen::Vector3d rBFf  = Hfw * ball.rBWw;

                // Desired position of robot on field
                Eigen::Vector3d rDFf = Eigen::Vector3d::Zero();

                // Check if the ball is in the bounding box
                if (rBFf.x() > box.x_min && rBFf.x() < box.x_max && rBFf.y() > box.y_min && rBFf.y() < box.y_max) {
                    // Do nothing as ball is inside of defending region, play normally
                    log<NUClear::DEBUG>("Ball is inside of bounding box");
                }
                else {
                    // If ball is in a region parallel and outside own bounding box of robot we clamp in the y
                    // direction and move to 1m behind ball
                    if (rBFf.x() >= 0 && rBFf.y() > box.y_min) {
                        log<NUClear::DEBUG>("Ball is in own half and outside bounding box");
                        // Clamp desired position to bounding box and try stay 1m behind ball
                        rDFf.x() = std::clamp(rBFf.x() + 1.0, box.x_min, box.x_max);
                        rDFf.y() = std::clamp(rBFf.y(), box.y_min, box.y_max);
                    }
                    else {
                        log<NUClear::DEBUG>("Ball is in opponents half and outside bounding box");
                        // Clamp desired position to bounding box
                        rDFf.x() = std::clamp(rBFf.x(), box.x_min, box.x_max);
                        rDFf.y() = std::clamp(rBFf.y(), box.y_min, box.y_max);
                    }

                    // Emit task to walk to desired position with heading facing opponents side of field
                    emit<Task>(std::make_unique<WalkToFieldPosition>(
                        pos_rpy_to_transform(Eigen::Vector3d(rDFf.x(), rDFf.y(), 0), Eigen::Vector3d(0, 0, -M_PI))));
                }
            });
    }

}  // namespace module::strategy
