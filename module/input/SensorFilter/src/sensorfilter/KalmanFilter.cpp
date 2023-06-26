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

#include "SensorFilter.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/BodySide.hpp"

#include "utility/actuation/ForwardKinematics.hpp"
#include "utility/input/LinkID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::input {

    using message::actuation::BodySide;

    using extension::Configuration;

    using utility::input::LinkID;
    using utility::input::ServoID;
    using utility::math::euler::EulerIntrinsicToMatrix;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::support::Expression;

    void SensorFilter::configure_kf(const Configuration& config) {
        // Kalman Filter Config
        cfg.Ac           = Eigen::Matrix<double, n_states, n_states>(config["kalman"]["Ac"].as<Expression>());
        cfg.C            = Eigen::Matrix<double, n_measurements, n_states>(config["kalman"]["C"].as<Expression>());
        cfg.Q.diagonal() = Eigen::VectorXd(config["kalman"]["Q"].as<Expression>());
        cfg.R.diagonal() = Eigen::VectorXd(config["kalman"]["R"].as<Expression>());
        // Initialise the Kalman filter
        kf.update(cfg.Ac, cfg.Bc, cfg.C, cfg.Q, cfg.R);
        kf.reset(Eigen::VectorXd::Zero(n_states), Eigen::MatrixXd::Identity(n_states, n_states));
        Hwt.translation() = Eigen::VectorXd(config["kalman"]["initial_rTWw"].as<Expression>());
        update_loop.enable();
    }

    void SensorFilter::update_odometry_kf(std::unique_ptr<Sensors>& sensors,
                                          const std::shared_ptr<const Sensors>& previous_sensors,
                                          const RawSensors& raw_sensors,
                                          const std::shared_ptr<const Stability>& stability,
                                          const std::shared_ptr<const WalkState>& walk_state) {
        // **************** Time Update ****************
        // Calculate our time offset from the last read then update the filter's time
        const double dt = std::max(
            std::chrono::duration_cast<std::chrono::duration<double>>(
                raw_sensors.timestamp - (previous_sensors ? previous_sensors->timestamp : raw_sensors.timestamp))
                .count(),
            0.0);

        // Integrate the walk command to estimate the change in position (x,y) and yaw orientation
        if (walk_state != nullptr && stability != nullptr) {
            integrate_walkcommand(dt, *stability, *walk_state);
        }

        // Integrate the rotational velocity to predict the change in orientation (roll, pitch)
        Eigen::Matrix<double, n_inputs, 1> u;
        kf.time(u, dt);

        // **************** Roll/Pitch Orientation Measurement Update ****************
        // Calculate the roll and pitch estimates from the accelerometer
        double est_roll  = std::atan2(sensors->accelerometer.y(), sensors->accelerometer.z());
        double est_pitch = std::atan2(-sensors->accelerometer.x(),
                                      std::sqrt(sensors->accelerometer.y() * sensors->accelerometer.y()
                                                + sensors->accelerometer.z() * sensors->accelerometer.z()));

        // Perform a gyroscope and accelerometer measurement based correction of the predicted roll and pitch
        Eigen::Matrix<double, n_measurements, 1> y;
        y << est_roll, est_pitch, sensors->gyroscope.x(), sensors->gyroscope.y();
        kf.measure(y);

        // Compute the height of the torso using the kinematics from a foot which is on the ground
        if (sensors->feet[BodySide::LEFT].down) {
            Hwt.translation().z() = Eigen::Isometry3d(sensors->Htx[LinkID::L_FOOT_BASE]).inverse().translation().z();
        }
        else if (sensors->feet[BodySide::RIGHT].down) {
            Hwt.translation().z() = Eigen::Isometry3d(sensors->Htx[LinkID::R_FOOT_BASE].inverse()).translation().z();
        }

        // **************** Construct Odometry Output ****************
        // Use the roll and pitch from the Kalman filter and the yaw from the dead reckoning of walk command
        const double roll  = kf.get_state()(0);
        const double pitch = kf.get_state()(1);
        Hwt.linear()       = EulerIntrinsicToMatrix(Eigen::Vector3d(roll, pitch, yaw));
        sensors->Htw       = Hwt.inverse().matrix();

        Eigen::Isometry3d Hwr = Eigen::Isometry3d::Identity();
        Hwr.linear()          = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Hwr.translation()     = Eigen::Vector3d(Hwt.translation().x(), Hwt.translation().y(), 0.0);
        sensors->Hrw          = Hwr.inverse().matrix();
    }
}  // namespace module::input
