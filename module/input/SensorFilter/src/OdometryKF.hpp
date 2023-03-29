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
 * Copyright 2023 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_INPUT_ODOMETRYKF_HPP
#define MODULE_INPUT_ODOMETRYKF_HPP

#include "SensorFilter.hpp"

namespace module::input {

    void SensorFilter::update_odometry_kf(std::unique_ptr<Sensors>& sensors,
                                          const std::shared_ptr<const Sensors>& previous_sensors,
                                          const RawSensors& raw_sensors) {

        // Calculate our time offset from the last read then update the filter's time
        const double dt = std::max(
            std::chrono::duration_cast<std::chrono::duration<double>>(
                raw_sensors.timestamp - (previous_sensors ? previous_sensors->timestamp : raw_sensors.timestamp))
                .count(),
            0.0);

        // Integrate the walk command to estimate the change in position and yaw orientation
        double dx = walk_command.x() * dt * cfg.deadreckoning_scale.x();
        double dy = walk_command.y() * dt * cfg.deadreckoning_scale.y();
        theta += walk_command.z() * dt * cfg.deadreckoning_scale.z();
        Hwt.translation().x() += dx * cos(theta) - dy * sin(theta);
        Hwt.translation().y() += dy * cos(theta) + dx * sin(theta);

        // Compute the height of the torso using the kinematics from a foot which is on the ground
        if (sensors->feet[BodySide::LEFT].down) {
            Hwt.translation().z() = Eigen::Isometry3d(sensors->Htx[ServoID::L_ANKLE_ROLL]).inverse().translation().z();
        }
        else if (sensors->feet[BodySide::RIGHT].down) {
            Hwt.translation().z() = Eigen::Isometry3d(sensors->Htx[ServoID::R_ANKLE_ROLL].inverse()).translation().z();
        }

        // **************** KF Time Update ****************
        // Integrate the state velocity to predict the change in orientation (roll, pitch)
        Eigen::Matrix<double, n_inputs, 1> u;
        kf.time(u, dt);

        // **************** KF Measurement Update ****************
        // Gyroscope and accelerometer measurement based correction of the predicted state
        double roll  = std::atan2(sensors->accelerometer.y(), sensors->accelerometer.z());
        double pitch = std::atan2(-sensors->accelerometer.x(),
                                  std::sqrt(sensors->accelerometer.y() * sensors->accelerometer.y()
                                            + sensors->accelerometer.z() * sensors->accelerometer.z()));

        Eigen::Matrix<double, n_measurements, 1> y;
        y << roll, pitch, sensors->gyroscope.x(), sensors->gyroscope.y();
        kf.measure(y);

        // **************** Construct Odometry Output (Htw) ****************
        const auto state = kf.get_state();  // [roll, pitch, roll_rate, pitch_rate, gyro_roll_bias, gyro_pitch_bias]
        Hwt.linear()     = EulerIntrinsicToMatrix(Eigen::Vector3d(state(0), state(1), theta));
        sensors->Htw     = Hwt.inverse().matrix();
    }
}  // namespace module::input
#endif  // MODULE_INPUT_ODOMETRYKF_HPP
