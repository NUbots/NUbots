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
#include "message/input/Sensors.hpp"
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
    using message::input::Sensors;
    using message::localisation::Field;
    using message::strategy::WalkToFieldPosition;

    using utility::math::euler::pos_rpy_to_transform;

    WalkInsideBoundedBox::WalkInsideBoundedBox(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {
        on<Configuration>("WalkInsideBoundedBox.yaml").then([this](const Configuration& config) {
            this->log_level         = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_search_timeout = duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["ball_search_timeout"].as<double>()));
            cfg.allow_pass_y_offset = config["allow_pass_y_offset"].as<double>();
            cfg.allow_pass_x_offset = config["allow_pass_x_offset"].as<double>();
        });

        on<Provide<WalkInsideBoundedBoxTask>, Optional<With<Ball>>, With<Field>, With<Sensors>>().then(
            [this](const WalkInsideBoundedBoxTask& box,
                   const std::shared_ptr<const Ball>& ball,
                   const Field& field,
                   const Sensors& sensors) {
                // Emit self as non-task for debugging
                emit(std::make_unique<WalkInsideBoundedBoxTask>(box));

                if (ball == nullptr || NUClear::clock::now() - ball->time_of_measurement > cfg.ball_search_timeout) {
                    log<NUClear::DEBUG>("Ball timeout. Returning to default position");
                    emit<Task>(std::make_unique<WalkToFieldPosition>(box.Hfd, true));
                    return;
                }

                log<NUClear::DEBUG>("Recent ball measurement");
                // Get the current position of the ball on the field
                Eigen::Vector3d rBFf = field.Hfw * ball->rBWw;
                // Desired position of robot on field
                Eigen::Vector3d rDFf = Eigen::Vector3d::Zero();
                // Check if the ball is in the bounding box
                if (rBFf.x() > box.x_min && rBFf.x() < box.x_max && rBFf.y() > box.y_min && rBFf.y() < box.y_max) {
                    // Do nothing as ball is inside of defending region, play normally
                    log<NUClear::DEBUG>("Ball is inside of bounding box");
                }
                else {
                    log<NUClear::DEBUG>("Ball is outside of bounding box");
                    // Clamp desired position to bounding box
                    rDFf.x() = std::clamp(rBFf.x(), box.x_min, box.x_max);
                    rDFf.y() = std::clamp(rBFf.y(), box.y_min, box.y_max);


                    double desired_heading = -M_PI;
                    if (rBFf.x() > box.x_max) {
                        // If the ball is in a region inbetween the bounding box and our own goal, add a y offset to
                        // allow pass from another robot
                        Eigen::Vector3d rDlFf = rDFf;
                        rDlFf.y()             = std::clamp(rBFf.y() + cfg.allow_pass_y_offset, box.y_min, box.y_max);
                        Eigen::Vector3d rDrFf = rDlFf;
                        rDrFf.y()             = std::clamp(rBFf.y() - cfg.allow_pass_y_offset, box.y_min, box.y_max);

                        // Select the desired position closest to the robot
                        Eigen::Isometry3d Hfr = field.Hfw * sensors.Hrw.inverse();
                        Eigen::Vector3d rRFf  = Hfr.translation();
                        if ((rDlFf - rRFf).norm() < (rDrFf - rRFf).norm()) {
                            rDFf = rDlFf;
                        }
                        else {
                            rDFf = rDrFf;
                        }

                        // Calculate the desired heading to face the ball
                        desired_heading = std::atan2(rBFf.y() - rDFf.y(), rBFf.x() - rDFf.x());
                        log<NUClear::DEBUG>("Ball is in pass region. Adding y offset");
                    }
                    else {
                        // If the ball is in a region inbetween the bounding box and opp goal, add an x offset to
                        // allow teammate to take ball
                        rDFf.x() += cfg.allow_pass_x_offset;
                    }

                    // Emit task to walk to desired position with heading facing opponents side of field
                    emit<Task>(std::make_unique<WalkToFieldPosition>(
                        pos_rpy_to_transform(Eigen::Vector3d(rDFf.x(), rDFf.y(), 0),
                                             Eigen::Vector3d(0, 0, desired_heading))));
                }
            });
    }

}  // namespace module::strategy
