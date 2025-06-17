/*
 * MIT License
 *
 * Copyright (c) 2013 NUbots
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

#include "SensorFilter.hpp"

#include "MotionModel.hpp"

#include "extension/Configuration.hpp"

#include "message/localisation/Field.hpp"

#include "utility/math/euler.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::input {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::localisation::ResetFieldLocalisation;

    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::math::filter::MahonyFilter;
    using utility::support::Expression;

    SensorFilter::SensorFilter(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("SensorFilter.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            // Button config
            cfg.button_debounce_threshold = config["buttons"]["debounce_threshold"].as<int>();

            // Foot down config
            cfg.foot_down.threshold = config["foot_down"]["threshold"].as<double>();
            cfg.foot_down.method    = config["foot_down"]["method"].as<std::string>();

            // Import URDF model
            nugus_model = tinyrobotics::import_urdf<double, n_servos>(config["urdf_path"].as<std::string>());

            // Configure the Mahony filter
            mahony_filter = MahonyFilter<double>(
                config["mahony"]["Kp"].as<Expression>(),
                config["mahony"]["Ki"].as<Expression>(),
                Eigen::Vector3d(config["mahony"]["initial_bias"].as<Expression>()),
                rpy_intrinsic_to_mat(Eigen::Vector3d(config["mahony"]["initial_rpy"].as<Expression>())));

            // Velocity filter config
            cfg.x_cut_off_frequency = config["velocity_low_pass"]["x_cut_off_frequency"].as<double>();
            cfg.y_cut_off_frequency = config["velocity_low_pass"]["y_cut_off_frequency"].as<double>();

            // Initialise the anchor frame (left foot base)
            Hwp.translation().y() = tinyrobotics::forward_kinematics<double, n_servos>(nugus_model,
                                                                                       nugus_model.home_configuration(),
                                                                                       std::string("left_foot_base"))
                                        .translation()
                                        .y();

            cfg.use_ground_truth = config["use_ground_truth"].as<bool>();
        });

        on<Startup>().then([this] {
            // Emit an initial walk state to ensure odometry starts if no other walk state is emitted
            emit(std::make_unique<WalkState>(message::behaviour::state::WalkState::State::UNKNOWN,
                                             Eigen::Vector3d::Zero(),
                                             message::behaviour::state::WalkState::Phase::DOUBLE));
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
        });

        on<Trigger<RawSensors>, Optional<With<Sensors>>, With<Stability>, Single, Priority::HIGH>().then(
            "Main Sensors Loop",
            [this](const RawSensors& raw_sensors,
                   const std::shared_ptr<const Sensors>& previous_sensors,
                   const Stability& stability) {
                auto sensors = std::make_unique<Sensors>();

                // Raw sensors (Accelerometer, Gyroscope, etc.)
                update_raw_sensors(sensors, previous_sensors, raw_sensors);

                // Kinematics (Htw, foot down, CoM, etc.)
                update_kinematics(sensors, raw_sensors);

                // Odometry (Htw and Hrw)
                update_odometry(sensors, previous_sensors, raw_sensors, stability);

                // Graph debug information
                if (log_level <= DEBUG) {
                    debug_sensor_filter(sensors, raw_sensors);
                }

                emit(std::move(sensors));
            });

        on<Last<20, Trigger<RawSensors>>>().then(
            [this](const std::list<std::shared_ptr<const RawSensors>>& raw_sensors) {
                // Detect wether a button has been pressed or not in the last 20 messages
                detect_button_press(raw_sensors);
            });

        on<Trigger<ResetFieldLocalisation>>().then([this] {
            // Reset anchor frame
            Hwp                   = Eigen::Isometry3d::Identity();
            Hwp.translation().y() = tinyrobotics::forward_kinematics<double, n_servos>(nugus_model,
                                                                                       nugus_model.home_configuration(),
                                                                                       std::string("left_foot_base"))
                                        .translation()
                                        .y();
        });
    }

}  // namespace module::input
