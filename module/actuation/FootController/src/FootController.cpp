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
#include "message/skill/ControlFoot.hpp"

namespace module::actuation {

    using extension::Configuration;

    using message::actuation::LeftLegIK;
    using message::actuation::RightLegIK;
    using message::skill::ControlLeftFoot;
    using message::skill::ControlRightFoot;

    FootController::FootController(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("FootController.yaml").then([this](const Configuration& config) {
            // Set log level from the configuration
            this->log_level   = config["log_level"].as<NUClear::LogLevel>();
            cfg.mode          = config["mode"].as<std::string>();
            cfg.desired_gains = config["servo_gains"].as<std::map<std::string, double>>();

            // Set gains of servo to startup phase values
            cfg.servo_states.clear();
            cfg.startup_gain = config["startup"]["servo_gain"].as<double>();
            for (const auto& servo : cfg.desired_gains) {
                utility::input::ServoID servo_id(servo.first);
                cfg.servo_states[servo_id] = ServoState(cfg.startup_gain, TORQUE_ENABLED);
            }

            // Balance config
            cfg.roll_p_gain     = config["balance"]["roll_p_gain"].as<double>();
            cfg.pitch_p_gain    = config["balance"]["pitch_p_gain"].as<double>();
            cfg.roll_i_gain     = config["balance"]["roll_i_gain"].as<double>();
            cfg.pitch_i_gain    = config["balance"]["pitch_i_gain"].as<double>();
            cfg.max_i_error     = config["balance"]["max_i_error"].as<double>();
            cfg.roll_d_gain     = config["balance"]["roll_d_gain"].as<double>();
            cfg.pitch_d_gain    = config["balance"]["pitch_d_gain"].as<double>();
            cfg.max_pitch_error = config["balance"]["max_pitch_error"].as<double>();
            cfg.max_roll_error  = config["balance"]["max_roll_error"].as<double>();

            // Emit request to set desired gains after a delay
            emit<Scope::DELAY>(std::make_unique<SetGains>(),
                               std::chrono::seconds(config["startup"]["duration"].as<int>()));
        });

        on<Trigger<SetGains>>().then([this] {
            for (const auto& [key, gain] : cfg.desired_gains) {
                utility::input::ServoID servo_id(key);
                cfg.servo_states[servo_id] = ServoState(gain, TORQUE_ENABLED);
            }
        });

        on<Provide<ControlLeftFoot>, With<Sensors>, Needs<LeftLegIK>, Priority::HIGH>().then(
            [this](const ControlLeftFoot& left_foot, const Sensors& sensors) {
                auto left_leg = std::make_unique<LeftLegIK>();

                control_foot(left_foot, left_leg, sensors, LimbID::LimbID::LEFT_LEG);

                // Emit IK tasks to achieve the desired pose
                emit<Task>(left_leg, 0, false, "Control left foot");
            });

        on<Provide<ControlRightFoot>, With<Sensors>, Needs<RightLegIK>, Priority::HIGH>().then(
            [this](const ControlRightFoot& right_foot, const Sensors& sensors) {
                auto right_leg = std::make_unique<RightLegIK>();

                control_foot(right_foot, right_leg, sensors, LimbID::RIGHT_LEG);

                // Emit IK tasks to achieve the desired pose
                emit<Task>(right_leg, 0, false, "Control right foot");
            });
    }

    // Conversion: Quaternion --> Fused angles (2D)
    void FootController::FusedFromQuat(const Eigen::Quaterniond& q, double& fusedPitch, double& fusedRoll) {
        // Calculate the fused pitch and roll
        double stheta = 2.0 * (q.y() * q.w() - q.x() * q.z());
        double sphi   = 2.0 * (q.y() * q.z() + q.x() * q.w());
        stheta        = (stheta >= 1.0 ? 1.0 : (stheta <= -1.0 ? -1.0 : stheta));  // Coerce stheta to [-1,1]
        sphi          = (sphi >= 1.0 ? 1.0 : (sphi <= -1.0 ? -1.0 : sphi));        // Coerce sphi   to [-1,1]
        fusedPitch    = asin(stheta);
        fusedRoll     = asin(sphi);
    }

    Eigen::Quaterniond FootController::QuatFromFused(double fusedPitch,
                                                     double fusedRoll)  // Assume: fusedYaw = 0, hemi = true
    {
        // Precalculate the sine values
        double sth  = sin(fusedPitch);
        double sphi = sin(fusedRoll);

        // Calculate the sine sum criterion
        double crit = sth * sth + sphi * sphi;

        // Calculate the tilt angle alpha
        double alpha   = (crit >= 1.0 ? M_PI_2 : acos(sqrt(1.0 - crit)));
        double halpha  = 0.5 * alpha;
        double chalpha = cos(halpha);
        double shalpha = sin(halpha);

        // Calculate the tilt axis angle gamma
        double gamma  = atan2(sth, sphi);
        double cgamma = cos(gamma);
        double sgamma = sin(gamma);

        // Return the required quaternion orientation (a rotation about (cgamma, sgamma, 0) by angle alpha)
        Eigen::Quaterniond result = Eigen::Quaterniond(chalpha, cgamma * shalpha, sgamma * shalpha, 0.0);
        return result;  // Order: (w,x,y,z)
    }

}  // namespace module::actuation
