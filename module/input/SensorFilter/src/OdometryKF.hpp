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

        // **************** KF Time Update ****************
        // Calculate our time offset from the last read then update the filter's time
        const double dt = std::max(
            std::chrono::duration_cast<std::chrono::duration<double>>(
                raw_sensors.timestamp - (previous_sensors ? previous_sensors->timestamp : raw_sensors.timestamp))
                .count(),
            0.0);

        // Integrate the walk command to get the change in position and orientation (x,y,theta)
        double delta_x   = walk_command.x() * dt;        // * cfg.scale_x;
        double delta_y   = walk_command.y() * dt;        // * cfg.scale_y;
        double delta_yaw = walk_command.z() * dt * 0.6;  // * cfg.scale_yaw;
        rRWw.x() += delta_x * cos(yaw + delta_yaw) - delta_y * sin(yaw + delta_yaw);
        rRWw.y() += delta_y * cos(yaw + delta_yaw) + delta_x * sin(yaw + delta_yaw);
        yaw += delta_yaw;

        // Integrate the state velocity to predict the change in orientation (roll, pitch)
        Eigen::Matrix<double, n_inputs, 1> u;
        pose_filter.time(u, dt);

        // **************** KF Measurement Update ****************
        // Gyroscope and accelerometer measurement based correction of the predicted state
        // Roll
        double roll = std::atan2(sensors->accelerometer.y(), sensors->accelerometer.z());
        // Pitch
        double pitch = std::atan2(-sensors->accelerometer.x(),
                                  std::sqrt(sensors->accelerometer.y() * sensors->accelerometer.y()
                                            + sensors->accelerometer.z() * sensors->accelerometer.z()));
        Eigen::Matrix<double, n_measurements, 1> y;
        y << roll, pitch;

        // Perform the correction step with accelerometer estimate of roll and pitch
        pose_filter.measure(y);

        // **************** Construct Odometry Output (Htw) ****************
        // Gives us the quaternion representation
        const auto state = pose_filter.get_state();  // [roll, pitch, roll_rate, pitch_rate, roll_bias, pitch_bias]
        // Construct the rotation matrix using roll, pitch and yaw
        Eigen::Matrix3d Rwt = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                              * Eigen::AngleAxisd(state(1), Eigen::Vector3d::UnitY())
                              * Eigen::AngleAxisd(state(0), Eigen::Vector3d::UnitX()).toRotationMatrix();
        // Construct the position using the dead reckoned position (x,y) of the robot in the world and kinematics
        // estimate of torso height
        if (sensors->feet[BodySide::LEFT].down) {
            auto Hft = sensors->Htx[ServoID::L_ANKLE_ROLL].inverse();
            z_height = Hft(2, 3);
        }
        else if (sensors->feet[BodySide::RIGHT].down) {
            auto Hft = sensors->Htx[ServoID::R_ANKLE_ROLL].inverse();
            z_height = Hft(2, 3);
        }

        Eigen::Vector3d rTWw(rRWw.x(), rRWw.y(), z_height);

        Eigen::Isometry3d Hwt;
        Hwt.linear()      = Rwt;
        Hwt.translation() = rTWw;
        sensors->Htw      = Hwt.inverse().matrix();
    }
}  // namespace module::input
#endif  // MODULE_INPUT_ODOMETRYKF_HPP
