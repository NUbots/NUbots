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
#include "AlignBallToGoal.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/strategy/AlignBallToGoal.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::planning::TurnAroundBall;
    using message::support::FieldDescription;
    using AlignBallToGoalTask = message::strategy::AlignBallToGoal;
    using utility::support::Expression;

    AlignBallToGoal::AlignBallToGoal(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("AlignBallToGoal.yaml").then([this](const Configuration& config) {
            // Use configuration here from file AlignBallToGoal.yaml
            this->log_level             = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_distance_threshold = config["ball_distance_threshold"].as<double>();
            cfg.angle_threshold         = config["angle_threshold"].as<Expression>();
        });

        on<Provide<AlignBallToGoalTask>, With<Ball>, With<Field>, With<Sensors>, With<FieldDescription>>().then(
            [this](const Ball& ball,
                   const Field& field,
                   const Sensors& sensors,
                   const FieldDescription& field_description) {
                // If the ball is close, align towards the goal
                Eigen::Vector3d rBRr    = sensors.Hrw * ball.rBWw;
                double distance_to_ball = rBRr.head(2).norm();
                if (distance_to_ball < cfg.ball_distance_threshold) {
                    // Get the robot's position (pose) on the field
                    Eigen::Isometry3d Hrf = sensors.Hrw * field.Hfw.inverse();

                    // Goal position relative to robot
                    Eigen::Vector3d rGFf = Eigen::Vector3d(-field_description.dimensions.field_length / 2.0, 0.0, 0.0);
                    Eigen::Vector3d rGRr = Hrf * rGFf;

                    // Find the angle to the goal - should be as close as possible to 0 to be aligned
                    double kick_angle = std::atan2(rGRr.y(), rGRr.x());

                    // Only align if we are not within a threshold of the goal
                    if (std::fabs(kick_angle) > cfg.angle_threshold) {
                        if (kick_angle < 0.0) {
                            emit<Task>(std::make_unique<TurnAroundBall>(true));
                        }
                        else {
                            emit<Task>(std::make_unique<TurnAroundBall>(false));
                        }
                    }
                }
            });
    }

}  // namespace module::strategy
