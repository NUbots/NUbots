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

#ifndef MODULE_INPUT_ODOMETRYMAHONY_HPP
#define MODULE_INPUT_ODOMETRYMAHONY_HPP

#include "SensorFilter.hpp"

namespace module::input {

    void SensorFilter::update_odometry_mahony(std::unique_ptr<Sensors>& sensors,
                                              const std::shared_ptr<const Sensors>& previous_sensors,
                                              const RawSensors& raw_sensors) {

        // **************** Time Update ****************
        // Calculate our time offset from the last read then update the filter's time
        const double dt = std::max(
            std::chrono::duration_cast<std::chrono::duration<double>>(
                raw_sensors.timestamp - (previous_sensors ? previous_sensors->timestamp : raw_sensors.timestamp))
                .count(),
            0.0);

        // Integrate the walk command to estimate the change in position and yaw orientation (x,y,theta)
        double dx     = walk_command.x() * dt * cfg.deadreckoning_scale.x();
        double dy     = walk_command.y() * dt * cfg.deadreckoning_scale.y();
        double dtheta = walk_command.z() * dt * cfg.deadreckoning_scale.z();
        theta += dtheta;
        Hwt.translation().x() += dx * cos(theta) - dy * sin(theta);
        Hwt.translation().y() += dy * cos(theta) + dx * sin(theta);

        // Compute the height of the torso using the kinematics from a foot which is on the ground
        if (sensors->feet[BodySide::LEFT].down) {
            Hwt.translation().z() = Eigen::Isometry3d(sensors->Htx[ServoID::L_ANKLE_ROLL]).inverse().translation().z();
        }
        else if (sensors->feet[BodySide::RIGHT].down) {
            Hwt.translation().z() = Eigen::Isometry3d(sensors->Htx[ServoID::R_ANKLE_ROLL].inverse()).translation().z();
        }

        // **************** Mahony Roll/Pitch Orientation Measurement Update ****************
        Eigen::Quaterniond quat_Rwt = Eigen::Quaterniond(Hwt.rotation());
        utility::math::filter::MahonyUpdate(sensors->accelerometer,
                                            sensors->gyroscope,
                                            dt,
                                            cfg.Ki,
                                            cfg.Kp,
                                            quat_Rwt,
                                            bias);
        // Extract the roll and pitch from the orientation quaternion
        Eigen::Vector3d rpy_mahony = MatrixToEulerIntrinsic(quat_Rwt.toRotationMatrix());

        // **************** Construct Odometry Output (Htw) ****************
        // Use the roll and pitch from the Mahony filter and the yaw from the dead reckoning walk command
        Hwt.linear() = EulerIntrinsicToMatrix(Eigen::Vector3d(rpy_mahony(0), rpy_mahony(1), theta));
        sensors->Htw = Hwt.inverse().matrix();
    }
}  // namespace module::input
#endif  // MODULE_INPUT_ODOMETRYKF_HPP
