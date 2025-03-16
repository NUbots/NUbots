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
#include "StrategiseLook.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/skill/Look.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/vision/Goal.hpp"

#include "utility/math/coordinates.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::skill::Look;
    using message::strategy::LookAtBall;
    using message::strategy::LookAtGoals;
    using message::vision::Goals;
    using utility::math::coordinates::sphericalToCartesian;

    StrategiseLook::StrategiseLook(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("StrategiseLook.yaml").then([this](const Configuration& config) {
            // Use configuration here from file StrategiseLook.yaml
            this->log_level         = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_search_timeout = duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["ball_search_timeout"].as<double>()));
            cfg.goal_search_timeout = duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["goal_search_timeout"].as<double>()));
        });

        // Trigger on Ball to update readings
        // Uses Every to update time difference so if the ball is not recent, the Look Task will not be emitted
        on<Provide<LookAtBall>, Trigger<Ball>, With<Sensors>>().then([this](const Ball& ball, const Sensors& sensors) {
            // If we have a ball and it is recent, look at it
            if (NUClear::clock::now() - ball.time_of_measurement < cfg.ball_search_timeout) {
                Eigen::Vector3d rBCc = ball.Hcw * ball.rBWw;
                Eigen::Vector3d rBCt = (sensors.Htw * ball.Hcw.inverse()).rotation() * rBCc;
                emit<Task>(std::make_unique<Look>(rBCt, true));
            }
        });

        // Trigger on Goals to update readings
        // Uses Every to update time difference so if the goals are not recent, the Look Task will not be emitted
        on<Provide<LookAtGoals>, Trigger<Goals>, With<Sensors>>().then(
            [this](const Goals& goals, const Sensors& sensors) {
                // If we have goals, with at least one measurement and the goals are recent, look at the goals
                if (!goals.goals.empty() && (NUClear::clock::now() - goals.timestamp < cfg.goal_search_timeout)) {
                    // Convert goal measurement to cartesian coordinates
                    Eigen::Vector3d rGCc = goals.goals[0].measurements[0].rGCc;
                    // Convert to torso space
                    Eigen::Vector3d rGCt = (sensors.Htw * goals.Hcw.inverse()).rotation() * rGCc;
                    // Look at the goal
                    emit<Task>(std::make_unique<Look>(rGCt, true));
                }
            });
    }

}  // namespace module::strategy
