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
#include "Look.hpp"

#include <algorithm>
#include <cmath>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/booster/BoosterHeadRot.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/Look.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::skill {

    using extension::Configuration;
    using message::booster::BoosterHeadRot;
    using message::input::Sensors;
    using utility::math::coordinates::screen_angular_from_object_direction;
    using utility::nusight::graph;
    using LookTask = message::skill::Look;

    Look::Look(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Look.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Look.yaml
            this->log_level      = config["log_level"].as<NUClear::LogLevel>();
            cfg.smoothing_factor = config["smoothing_factor"].as<float>();
            cfg.max_hold_yaw     = config["max_hold_yaw"].as<double>();
        });

        on<Start<LookTask>>().then([this] { have_look = false; });

        on<Provide<LookTask>, Every<90, Per<std::chrono::seconds>>, Optional<With<Sensors>>>().then(
            [this](const LookTask& look, const std::shared_ptr<const Sensors>& sensors) {
                // Normalise the look vector
                Eigen::Vector3d req_uPCt = look.rPCt.normalized();

                // Hold the direction's yaw in the world: turn it back by how far the robot has turned since it was
                // given
                if (look.hold_yaw && sensors != nullptr) {
                    const Eigen::Matrix3d Rwr = sensors->Hrw.linear().transpose();
                    const double robot_yaw    = std::atan2(Rwr(1, 0), Rwr(0, 0));
                    if (!have_look || look.rPCt != last_rPCt) {
                        hold_reference_yaw = robot_yaw;
                    }
                    const double turned = std::remainder(robot_yaw - hold_reference_yaw, 2.0 * M_PI);
                    req_uPCt            = Eigen::AngleAxisd(-turned, Eigen::Vector3d::UnitZ()) * req_uPCt;
                }
                have_look = true;
                last_rPCt = look.rPCt;

                // If switching from non-smoothed to smoothed angle command, reset the initial goal angle to help
                // locking on to the target
                if (smooth == false && look.smooth == true) {
                    uPCt = req_uPCt;
                }
                smooth = look.smooth;

                // If smoothing requested, smooth requested angles with exponential filter
                uPCt = smooth ? (cfg.smoothing_factor * req_uPCt + (1 - cfg.smoothing_factor) * uPCt) : req_uPCt;

                // Convert the look direction into head yaw/pitch angles the K1 SDK understands
                // Returns {atan2(y, x), atan2(z, x)} = {yaw, pitch}
                Eigen::Vector2d rot = screen_angular_from_object_direction(uPCt);
                rot.y()             = -rot.y();  // swap signs for SDK
                if (look.hold_yaw) {
                    rot.x() = std::clamp(rot.x(), -cfg.max_hold_yaw, cfg.max_hold_yaw);
                }

                // Send the head rotation command to the Booster SDK (handled by Booster HardwareIO)
                emit(std::make_unique<BoosterHeadRot>(rot));

                if (log_level <= NUClear::LogLevel::DEBUG) {
                    emit(graph("Look head rot (yaw, pitch)", rot.x(), rot.y()));
                }
            });
    }

}  // namespace module::skill
