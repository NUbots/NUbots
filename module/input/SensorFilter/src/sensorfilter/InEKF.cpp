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

#include "utility/math/filter/inekf/InEKF.hpp"

#include "SensorFilter.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/BodySide.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::input {

    using message::actuation::BodySide;

    using extension::Configuration;

    using utility::input::ServoID;
    using utility::math::euler::EulerIntrinsicToMatrix;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::nusight::graph;
    using utility::support::Expression;
    void SensorFilter::configure_inekf(const Configuration& config) {
        // IEKF Filter Config
        // SET INITIAL PARAMETERS FOR THE INEKF

        cfg.inekf.initial_position    = config["inekf"]["initial_state"]["position"].as<Expression>();
        cfg.inekf.initial_orientation = config["inekf"]["initial_state"]["orientation"].as<Expression>();
        cfg.inekf.initial_velocity    = config["inekf"]["initial_state"]["velocity"].as<Expression>();
        cfg.inekf.initial_gyro_bias   = config["inekf"]["initial_state"]["gyro_bias"].as<Expression>();
        cfg.inekf.initial_acc_bias    = config["inekf"]["initial_state"]["acc_bias"].as<Expression>();

        utility::math::filter::inekf::RobotState initial_state{};
        initial_state.set_position(cfg.inekf.initial_position);
        initial_state.set_rotation(cfg.inekf.initial_orientation);
        initial_state.set_velocity(cfg.inekf.initial_velocity);
        initial_state.set_gyroscope_bias(cfg.inekf.initial_gyro_bias);
        initial_state.set_accelerometer_bias(cfg.inekf.initial_acc_bias);

        inekf_filter.set_state(initial_state);

        // SET NOISE PARAMETERS FOR THE INEKF
        cfg.inekf.noise_gyro         = config["inekf"]["noise"]["gyro"].as<double>();
        cfg.inekf.noise_acc          = config["inekf"]["noise"]["acc"].as<double>();
        cfg.inekf.noise_gyro_bias    = config["inekf"]["noise"]["gyro_bias"].as<double>();
        cfg.inekf.noise_acc_bias     = config["inekf"]["noise"]["acc_bias"].as<double>();
        cfg.inekf.noise_foot_sensors = config["inekf"]["noise"]["foot_sensors"].as<double>();

        utility::math::filter::inekf::NoiseParams noise_params(cfg.inekf.noise_gyro,
                                                               cfg.inekf.noise_acc,
                                                               cfg.inekf.noise_gyro_bias,
                                                               cfg.inekf.noise_acc_bias,
                                                               cfg.inekf.noise_foot_sensors);

        inekf_filter.set_noise_params(noise_params);
        log<NUClear::DEBUG>("InEKF Configured");
    }

    void SensorFilter::update_odometry_inekf(std::unique_ptr<Sensors>& sensors,
                                             const std::shared_ptr<const Sensors>& previous_sensors,
                                             const RawSensors& raw_sensors) {
        // **************** Time Update ****************
        const double dt = std::max(
            std::chrono::duration_cast<std::chrono::duration<double>>(
                raw_sensors.timestamp - (previous_sensors ? previous_sensors->timestamp : raw_sensors.timestamp))
                .count(),
            0.0);

        inekf_filter.propagate(sensors->gyroscope, sensors->accelerometer, dt);

        // **************** Roll/Pitch Orientation Measurement Update ****************
        // Contact data for filter
        inekf_filter.set_contacts(std::vector<std::pair<int, bool>>{{0, sensors->feet[BodySide::RIGHT].down},
                                                                    {1, sensors->feet[BodySide::LEFT].down}});

        // Kinematics data for filter
        // Calculate the average length of both legs from the torso and accumulate this measurement
        Eigen::Isometry3d Htr = Eigen::Isometry3d(sensors->Htx[ServoID::L_FOOT_BASE]);
        Eigen::Isometry3d Htl = Eigen::Isometry3d(sensors->Htx[ServoID::R_FOOT_BASE]);
        utility::math::filter::inekf::kinematics measured_kinematics;
        measured_kinematics.emplace_back(
            utility::math::filter::inekf::KinematicPose{0,
                                                        Htr.inverse().matrix(),
                                                        Eigen::Matrix<double, 6, 6>::Zero()});
        measured_kinematics.emplace_back(
            utility::math::filter::inekf::KinematicPose{1,
                                                        Htl.inverse().matrix(),
                                                        Eigen::Matrix<double, 6, 6>::Zero()});
        inekf_filter.correct_kinematics(measured_kinematics);

        // **************** Construct Odometry Output ****************
        Eigen::Isometry3d Hwt;
        Hwt.linear()      = inekf_filter.get_state().get_rotation();
        Hwt.translation() = inekf_filter.get_state().get_position();
        sensors->Htw      = Hwt.inverse().matrix();

        Eigen::Isometry3d Hwr = Eigen::Isometry3d::Identity();
        // Hwr.linear()          = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        // Hwr.translation()     = Eigen::Vector3d(Hwt.translation().x(), Hwt.translation().y(), 0.0);
        sensors->Hrw = Hwr.inverse();
        update_loop.enable();
    }
}  // namespace module::input
