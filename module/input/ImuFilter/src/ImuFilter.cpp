/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "ImuFilter.hpp"

#include "extension/Configuration.hpp"

#include "message/input/Imu.hpp"
#include "message/motion/BodySide.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"
#include "utility/motion/ForwardKinematics.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::input {

    using extension::Configuration;

    using message::input::Imu;
    using message::platform::RawSensors;

    using utility::nusight::graph;
    using utility::support::Expression;

    ImuFilter::ImuFilter(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("ImuFilter.yaml").then([this](const Configuration& cfg) {
            log_level = cfg["log_level"].as<NUClear::LogLevel>();

            // Define the state transition model
            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_states, n_states);
            A << 0, 0, 0, 1, 0, 0, 0, 0, 0;

            // Define the control model
            Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n_states, n_inputs);

            // Define the measurement model
            Eigen::MatrixXd C = Eigen::MatrixXd::Zero(n_outputs, n_states);
            C << 0, 1, 0, 1, 0, 1;

            // Define the process noise covariance TODO: Move to yaml
            Eigen::Matrix3d Q;
            Q << 3.336564728582061e-04, 1.210220716654972e-05, -1.708092671728623e-04, 1.210220716654972e-05,
                4.680081577954583e-06, 6.268022135995622e-06, -1.708092671728623e-04, 6.268022135995622e-06,
                1.320010202073407e-04;

            // Define the measurement noise covariance TODO: Move to yaml
            Eigen::Matrix2d R;
            R << 0.0502, 0.0, 0.0, 3.4546e-06;

            // Update filter model
            filter.update_model(A, B, C, Q, R);
        });

        on<Startup>().then([this] {
            // Set initial state
            filter.init(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());

            // Set the initial update time
            last_update_time = NUClear::clock::now();
        });

        on<Trigger<RawSensors>, Single, Priority::HIGH>().then("Main Loop", [this](const RawSensors& input) {
            auto imu = std::make_unique<Imu>();

            // Get the timestamp
            imu->timestamp = input.timestamp;

            // Get the accelerometer data and angle from the raw sensors
            imu->accelerometer = input.accelerometer.cast<double>();
            double pitch_angle = std::atan2(imu->accelerometer.x(), imu->accelerometer.z());

            // Get the gyroscope data
            imu->gyroscope = input.gyroscope.cast<double>();

            // Get the current time
            NUClear::clock::time_point current_time = NUClear::clock::now();

            // Calculate the time since the last update in seconds
            double dt = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_update_time).count();

            // Run the filter
            Eigen::Matrix<double, n_inputs, 1> u = Eigen::Matrix<double, n_inputs, 1>::Zero();
            Eigen::Matrix<double, n_outputs, 1> measurements;
            measurements(0) = pitch_angle;
            measurements(1) = imu->gyroscope.y();
            filter.run(u, measurements, dt);

            // Get the state
            Eigen::Vector3d state = filter.get_state();

            // Plot the filtered pitch angle
            emit(graph("Raw Pitch Angle", pitch_angle));
            emit(graph("Filtered Pitch Angle", state(1)));

            // Update time of last update for next update
            last_update_time = current_time;
        });
    }
}  // namespace module::input
