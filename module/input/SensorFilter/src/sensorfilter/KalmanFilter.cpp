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
#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::input {

    using message::actuation::BodySide;

    using extension::Configuration;

    using utility::actuation::kinematics::calculateGroundSpace;
    using utility::input::ServoID;
    using utility::math::euler::EulerIntrinsicToMatrix;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::support::Expression;

    void SensorFilter::configure_kf(const Configuration& config) {
        // Kalman Filter Config
        cfg.Ac           = Eigen::Matrix<float, n_states, n_states>(config["kalman"]["Ac"].as<Expression>());
        cfg.C            = Eigen::Matrix<float, n_measurements, n_states>(config["kalman"]["C"].as<Expression>());
        cfg.Q.diagonal() = Eigen::VectorXf(config["kalman"]["Q"].as<Expression>());
        cfg.R.diagonal() = Eigen::VectorXf(config["kalman"]["R"].as<Expression>());
        // Initialise the Kalman filter
        kf.update(cfg.Ac, cfg.Bc, cfg.C, cfg.Q, cfg.R);
        kf.reset(Eigen::VectorXf::Zero(n_states), Eigen::MatrixXf::Identity(n_states, n_states));
        Hwt.translation() = Eigen::VectorXf(config["kalman"]["initial_rTWw"].as<Expression>());
        update_loop.enable();
    }

    void SensorFilter::update_odometry_kf(std::unique_ptr<Sensors>& sensors,
                                          const std::shared_ptr<const Sensors>& previous_sensors,
                                          const RawSensors& raw_sensors) {
        // **************** Time Update ****************
        // Calculate our time offset from the last read then update the filter's time
        const float dt = std::max(
            std::chrono::duration_cast<std::chrono::duration<float>>(
                raw_sensors.timestamp - (previous_sensors ? previous_sensors->timestamp : raw_sensors.timestamp))
                .count(),
            0.0f);

        // Integrate the walk command to estimate the change in position (x,y) and yaw orientation
        integrate_walkcommand(dt);

        // Integrate the rotational velocity to predict the change in orientation (roll, pitch)
        Eigen::Matrix<float, n_inputs, 1> u;
        kf.time(u, dt);

        // **************** Roll/Pitch Orientation Measurement Update ****************
        // Calculate the roll and pitch estimates from the accelerometer
        float est_roll  = std::atan2(sensors->accelerometer.y(), sensors->accelerometer.z());
        float est_pitch = std::atan2(-sensors->accelerometer.x(),
                                     std::sqrt(sensors->accelerometer.y() * sensors->accelerometer.y()
                                               + sensors->accelerometer.z() * sensors->accelerometer.z()));

        // Perform a gyroscope and accelerometer measurement based correction of the predicted roll and pitch
        Eigen::Matrix<float, n_measurements, 1> y;
        y << est_roll, est_pitch, sensors->gyroscope.x(), sensors->gyroscope.y();
        kf.measure(y);

        // Update the height of the torso using the kinematics from a foot which is on the ground
        if (sensors->feet[BodySide::LEFT].down) {
            Hwt.translation().z() = Eigen::Isometry3f(sensors->Htx[ServoID::L_ANKLE_ROLL]).inverse().translation().z();
        }
        else if (sensors->feet[BodySide::RIGHT].down) {
            Hwt.translation().z() = Eigen::Isometry3f(sensors->Htx[ServoID::R_ANKLE_ROLL].inverse()).translation().z();
        }

        // **************** Construct Odometry Output ****************
        // Use the roll and pitch from the Kalman filter and the yaw from the dead reckoning of walk command
        const float roll  = kf.get_state()(0);
        const float pitch = kf.get_state()(1);
        Hwt.linear()      = EulerIntrinsicToMatrix(Eigen::Vector3f(roll, pitch, yaw));
        sensors->Htw      = Hwt.inverse().matrix();

        Eigen::Isometry3f Hwr = Eigen::Isometry3f::Identity();
        Hwr.linear()          = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        Hwr.translation()     = Eigen::Vector3f(Hwt.translation().x(), Hwt.translation().y(), 0.0f);
        sensors->Hrw          = Hwr.inverse().matrix();
    }
}  // namespace module::input
