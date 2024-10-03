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
#include "DiveToBall.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/BodySide.hpp"
#include "message/actuation/Limbs.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/skill/Dive.hpp"
#include "message/strategy/DiveToBall.hpp"

namespace module::strategy {

    using extension::Configuration;
    using DiveToBallTask = message::strategy::DiveToBall;
    using message::actuation::BodySequence;
    using message::actuation::BodySide;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::skill::Dive;

    DiveToBall::DiveToBall(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("DiveToBall.yaml").then([this](const Configuration& config) {
            // Use configuration here from file DiveToBall.yaml
            this->log_level               = config["log_level"].as<NUClear::LogLevel>();
            cfg.diving_distance_threshold = config["diving_distance_threshold"].as<float>();
        });

        on<Provide<DiveToBallTask>, Trigger<Ball>, With<Sensors>>().then(
            [this](const RunInfo& info, const Ball& ball, const Sensors& sensors) {
                // If we ran because the Dive is done, then we don't keep running the Dive
                if (info.run_reason == RunInfo::RunReason::SUBTASK_DONE) {
                    return;
                }
                Eigen::Vector3d rBRr = sensors.Hrw * ball.rBWw;
                // If the distance to the ball is less than the threshold, dive
                if (std::abs(rBRr.x()) < cfg.diving_distance_threshold) {
                    // Determine angle to ball and whether we should dive right or left
                    double yaw_angle        = std::atan2(rBRr.y(), rBRr.x());
                    BodySide dive_direction = yaw_angle < 0 ? BodySide::RIGHT : BodySide::LEFT;
                    emit<Task>(std::make_unique<Dive>(dive_direction));
                }
            });
    }

}  // namespace module::strategy
