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
#include "WalkToFieldPosition.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::planning::TurnOnSpot;
    using message::planning::WalkTo;
    using message::strategy::StandStill;
    using WalkToFieldPositionTask = message::strategy::WalkToFieldPosition;
    using utility::math::euler::mat_to_rpy_intrinsic;

    WalkToFieldPosition::WalkToFieldPosition(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("WalkToFieldPosition.yaml").then([this](const Configuration& config) {
            // Use configuration here from file WalkToFieldPosition.yaml
            this->log_level               = config["log_level"].as<NUClear::LogLevel>();
            cfg.stop_tolerance_position   = config["stop_tolerance"]["position"].as<double>();
            cfg.stop_tolerance_heading    = config["stop_tolerance"]["heading"].as<double>();
            cfg.resume_tolerance_position = config["resume_tolerance"]["position"].as<double>();
            cfg.resume_tolerance_heading  = config["resume_tolerance"]["heading"].as<double>();
        });

        on<Provide<WalkToFieldPositionTask>, With<Field>, With<Sensors>, Every<30, Per<std::chrono::seconds>>>().then(
            [this](const WalkToFieldPositionTask& walk_to_field_position, const Field& field, const Sensors& sensors) {
                // Get the transformation from robot {r} space to field {f} space
                const Eigen::Isometry3d Hfr = field.Hfw * sensors.Hrw.inverse();

                Eigen::Isometry3d Hfp = Eigen::Isometry3d::Identity();
                Hfp.translation()     = Eigen::Vector3d(walk_to_field_position.rPFf);
                Hfp.linear() =
                    Eigen::AngleAxisd(walk_to_field_position.heading, Eigen::Vector3d::UnitZ()).toRotationMatrix();

                // Transform from desired field position into robot space
                Eigen::Isometry3d Hrp = Hfr.inverse() * Hfp;

                // Compute the current position error and heading error in field {f} space
                const double position_error = Hrp.translation().norm();
                const double heading_error  = std::abs(mat_to_rpy_intrinsic(Hrp.linear()).z());
                log<NUClear::DEBUG>("Position error: ", position_error, " Heading error: ", heading_error);

                // If we have stopped but our position and heading error is now above resume tolerance, resume
                // walking
                if (stopped
                    && (position_error > cfg.resume_tolerance_position
                        || heading_error > cfg.resume_tolerance_heading)) {
                    stopped = false;
                }

                // If the error in the desired field position and heading is low enough, don't do anything
                if (!stopped && position_error < cfg.stop_tolerance_position
                    && heading_error < cfg.stop_tolerance_heading) {
                    stopped = true;
                }

                // If we are stopped, do nothing
                if (stopped) {
                    return;
                }

                // If we are far from position and desired heading, walk directly to the field position
                if (position_error > cfg.stop_tolerance_position && heading_error > cfg.stop_tolerance_heading) {
                    const double desired_heading = std::atan2(Hrp.translation().y(), Hrp.translation().x());
                    emit<Task>(std::make_unique<WalkTo>(Hrp.translation(), desired_heading, Hrp));
                    emit(std::make_unique<WalkTo>(Hrp.translation(), desired_heading, Hrp));
                    log<NUClear::DEBUG>("Walking to desired position");
                }
                // If we are close to desired position but not orientated correctly, rotate in place
                else if (position_error < cfg.stop_tolerance_position && heading_error > cfg.stop_tolerance_heading) {
                    // Get the desired heading in robot space
                    const double desired_heading = mat_to_rpy_intrinsic(Hrp.linear()).z();
                    bool clockwise               = desired_heading < 0;
                    emit<Task>(std::make_unique<TurnOnSpot>(clockwise));
                    log<NUClear::DEBUG>("Rotating in place to desired heading");
                }
                // Otherwise,do nothing
                else {
                    log<NUClear::DEBUG>("At desired position, stopping");
                }
            });
    }

}  // namespace module::strategy
