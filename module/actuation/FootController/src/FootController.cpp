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
#include "FootController.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/LimbsIK.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/ControlFoot.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::actuation {

    using extension::Configuration;

    using message::actuation::LeftLegIK;
    using message::actuation::RightLegIK;
    using message::actuation::ServoState;
    using message::input::Sensors;
    using message::skill::ControlLeftFoot;
    using message::skill::ControlRightFoot;

    using utility::input::LimbID;

    FootController::FootController(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("FootController.yaml").then([this](const Configuration& config) {
            // Use configuration here from file FootController.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.servo_gain  = config["servo_gain"].as<double>();
            cfg.keep_level  = config["keep_level"].as<bool>();
        });

        on<Provide<ControlLeftFoot>, With<Sensors>, Needs<LeftLegIK>>().then(
            [this](const ControlLeftFoot& left_foot, const Sensors& sensors) {
                // Construct Leg IK tasks
                auto left_leg  = std::make_unique<LeftLegIK>();
                left_leg->time = left_foot.time;
                if (left_foot.keep_level && cfg.keep_level) {
                    // Calculate the desired foot orientation to keep the foot level with the ground
                    Eigen::Isometry3d Htr           = sensors.Htw * sensors.Hrw.inverse();
                    Eigen::Isometry3d Htf_corrected = left_foot.Htf;
                    Htf_corrected.linear()          = Htr.linear();
                    left_leg->Htl                   = Htf_corrected;
                }
                else {
                    left_leg->Htl = left_foot.Htf;
                }

                for (auto id : utility::input::LimbID::servos_for_limb(LimbID::LEFT_LEG)) {
                    left_leg->servos[id] = ServoState(cfg.servo_gain, 100);
                }

                // Emit IK tasks to achieve the desired pose
                emit<Task>(left_leg, 0, false, "Control left foot");
            });

        on<Provide<ControlRightFoot>, With<Sensors>, Needs<RightLegIK>>().then(
            [this](const ControlRightFoot& right_foot, const Sensors& sensors) {
                auto right_leg  = std::make_unique<RightLegIK>();
                right_leg->time = right_foot.time;

                if (right_foot.keep_level && cfg.keep_level) {
                    // Calculate the desired foot orientation to keep the foot level with the ground
                    Eigen::Isometry3d Htr           = sensors.Htw * sensors.Hrw.inverse();
                    Eigen::Isometry3d Htf_corrected = right_foot.Htf;
                    Htf_corrected.linear()          = Htr.linear();
                    right_leg->Htr                  = Htf_corrected;
                }
                else {
                    right_leg->Htr = right_foot.Htf;
                }

                for (auto id : utility::input::LimbID::servos_for_limb(LimbID::RIGHT_LEG)) {
                    right_leg->servos[id] = ServoState(cfg.servo_gain, 100);
                }

                // Emit IK tasks to achieve the desired pose
                emit<Task>(right_leg, 0, false, "Control right foot");
            });
    }

}  // namespace module::actuation
