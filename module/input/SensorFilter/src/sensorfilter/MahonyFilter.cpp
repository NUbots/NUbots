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

#include "utility/math/filter/MahonyFilter.hpp"

#include "SensorFilter.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/BodySide.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::input {

    using message::actuation::BodySide;

    using extension::Configuration;

    using utility::input::ServoID;
    using utility::math::euler::EulerIntrinsicToMatrix;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::math::filter::MahonyUpdate;
    using utility::support::Expression;

    void SensorFilter::integrate_walkcommand(const double dt) {
        // Integrate the walk command to estimate the change in position and yaw orientation
        double dx = walk_command.x() * dt * cfg.deadreckoning_scale.x();
        double dy = walk_command.y() * dt * cfg.deadreckoning_scale.y();
        yaw += walk_command.z() * dt * cfg.deadreckoning_scale.z();
        // Rotate the change in position into world coordinates before adding it to the current position
        Hwt.translation().x() += dx * cos(yaw) - dy * sin(yaw);
        Hwt.translation().y() += dy * cos(yaw) + dx * sin(yaw);
    }

    void SensorFilter::configure_mahony(const Configuration& config) {
        // Mahony Filter Config
        cfg.Ki            = config["mahony"]["Ki"].as<Expression>();
        cfg.Kp            = config["mahony"]["Kp"].as<Expression>();
        cfg.bias          = Eigen::Vector3d(config["mahony"]["bias"].as<Expression>());
        Hwt.translation() = Eigen::VectorXd(config["mahony"]["initial_rTWw"].as<Expression>());

        // Optimisation Config
        cfg.run_optimisation = config["mahony"]["run_optimisation"].as<bool>();
        cfg.min_gain         = config["mahony"]["min_gain"].as<Expression>();
        cfg.max_gain         = config["mahony"]["max_gain"].as<Expression>();
        cfg.num_grid_points  = config["mahony"]["num_grid_points"].as<int>();
    }

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

        // Integrate the walk command to estimate the change in position (x,y) and yaw orientation
        integrate_walkcommand(dt);

        // **************** Roll/Pitch Orientation Measurement Update ****************
        utility::math::filter::MahonyUpdate(sensors->accelerometer,
                                            sensors->gyroscope,
                                            Hwt,
                                            dt,
                                            cfg.Ki,
                                            cfg.Kp,
                                            cfg.bias);
        // Extract the roll and pitch from the orientation quaternion
        Eigen::Vector3d rpy = MatrixToEulerIntrinsic(Hwt.rotation());

        // Compute the height of the torso using the kinematics from a foot which is on the ground
        if (sensors->feet[BodySide::LEFT].down) {
            Hwt.translation().z() = Eigen::Isometry3d(sensors->Htx[ServoID::L_ANKLE_ROLL]).inverse().translation().z();
        }
        else if (sensors->feet[BodySide::RIGHT].down) {
            Hwt.translation().z() = Eigen::Isometry3d(sensors->Htx[ServoID::R_ANKLE_ROLL].inverse()).translation().z();
        }

        // **************** Construct Odometry Output (Htw) ****************
        // Use the roll and pitch from the Mahony filter and the yaw from the dead reckoning of walk command
        const double roll  = rpy(0);
        const double pitch = rpy(1);
        Hwt.linear()       = EulerIntrinsicToMatrix(Eigen::Vector3d(roll, pitch, yaw));
        sensors->Htw       = Hwt.inverse().matrix();
        update_loop.enable();
    }

    void SensorFilter::optimize_mahony(const std::list<std::shared_ptr<const Sensors>>& sensors_history,
                                       const std::list<std::shared_ptr<const RawSensors>>& raw_sensors_history,
                                       double& Kp,
                                       double& Ki,
                                       const double min_gain,
                                       const double max_gain,
                                       const int num_grid_points) {
        double min_avg_cost = std::numeric_limits<double>::max();
        double K_step       = (max_gain - min_gain) / num_grid_points;

        for (int i = 0; i <= num_grid_points; ++i) {
            for (int j = 0; j <= num_grid_points; ++j) {
                double test_Kp = min_gain + i * K_step;
                double test_Ki = min_gain + j * K_step;

                double total_cost = 0;
                int count         = 0;

                auto sensor_iter     = sensors_history.rbegin();
                auto raw_sensor_iter = raw_sensors_history.rbegin();

                for (; sensor_iter != sensors_history.rend() && raw_sensor_iter != raw_sensors_history.rend();
                     ++sensor_iter, ++raw_sensor_iter) {
                    const std::shared_ptr<const Sensors>& sensors        = *sensor_iter;
                    const std::shared_ptr<const RawSensors>& raw_sensors = *raw_sensor_iter;

                    // Compute dt (time since last sensor reading)
                    auto prev_raw_sensor_iter = std::next(raw_sensor_iter);
                    double dt                 = 0.0;
                    if (prev_raw_sensor_iter != raw_sensors_history.rend()) {
                        dt = std::max(std::chrono::duration_cast<std::chrono::duration<double>>(
                                          raw_sensors->timestamp - (*prev_raw_sensor_iter)->timestamp)
                                          .count(),
                                      0.0);
                    }

                    // Get ground truth and estimated orientation
                    Eigen::Isometry3d true_Hwt = Eigen::Isometry3d(raw_sensors->odometry_ground_truth.Htw).inverse();
                    Eigen::Isometry3d Hwt_temp = Eigen::Isometry3d(sensors->Htw).inverse();
                    Eigen::Vector3d bias_temp  = cfg.bias;

                    // Apply Mahony update
                    MahonyUpdate(sensors->accelerometer, sensors->gyroscope, Hwt_temp, dt, test_Ki, test_Kp, bias_temp);

                    // Compute cost for this measurement
                    double cost = cost_function_mahony(Hwt_temp, true_Hwt);
                    total_cost += cost;
                    count++;
                }

                double avg_cost = total_cost / count;

                if (avg_cost < min_avg_cost) {
                    min_avg_cost = avg_cost;
                    Kp           = test_Kp;
                    Ki           = test_Ki;
                }
            }
        }
    }

    double SensorFilter::cost_function_mahony(const Eigen::Isometry3d& Hwt, const Eigen::Isometry3d& true_Hwt) {
        Eigen::Vector3d rpy      = MatrixToEulerIntrinsic(Hwt.rotation());
        Eigen::Vector3d true_rpy = MatrixToEulerIntrinsic(true_Hwt.rotation());
        double cost              = (rpy.head<2>() - true_rpy.head<2>()).norm();
        return cost;
    }

}  // namespace module::input
