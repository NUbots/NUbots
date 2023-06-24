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

#include "message/actuation/BodySide.hpp"
#include "message/motion/GetupCommand.hpp"
#include "message/motion/WalkCommand.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::input {

    using message::actuation::BodySide;
    using utility::input::ServoID;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::nusight::graph;
    using utility::support::Expression;

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::ExecuteGetup;
    using message::motion::KillGetup;
    using message::motion::StopCommand;
    using message::motion::WalkCommand;

    using extension::Configuration;

    SensorFilter::SensorFilter(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("SensorFilter.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            // **************************************** Button config ****************************************
            cfg.buttons.debounce_threshold = config["buttons"]["debounce_threshold"].as<int>();

            // **************************************** Foot down config ****************************************
            const FootDownMethod method = config["foot_down"]["method"].as<std::string>();
            std::map<FootDownMethod, float> thresholds;
            for (const auto& threshold : config["foot_down"]["known_methods"]) {
                thresholds[threshold["name"].as<std::string>()] = threshold["certainty_threshold"].as<float>();
            }
            cfg.footDown.set_method(method, thresholds);

            //  **************************************** Configure Filters ****************************************
            cfg.filtering_method = config["filtering_method"].as<std::string>();
            configure_ukf(config);
            configure_kf(config);
            configure_mahony(config);

            // Deadreckoning
            cfg.deadreckoning_scale = Eigen::Vector3d(config["deadreckoning_scale"].as<Expression>());
        });

        on<Last<20, Trigger<RawSensors>>, Single>().then(
            [this](const std::list<std::shared_ptr<const RawSensors>>& sensors) {
                // If we need to reset the UKF filter, do that here
                if (reset_filter.load()) {
                    // We have finished resetting the filter now
                    switch (ukf.reset(cfg.initial_mean.getStateVec(), cfg.initial_covariance.asDiagonal())) {
                        case Eigen::Success:
                            log<NUClear::INFO>("Motion Model UKF has been reset");
                            reset_filter.store(false);
                            update_loop.enable();
                            break;
                        case Eigen::NumericalIssue:
                            log<NUClear::WARN>(
                                "Cholesky decomposition failed. The provided data did not satisfy the prerequisites.");
                            break;
                        case Eigen::NoConvergence:
                            log<NUClear::WARN>("Cholesky decomposition failed. Iterative procedure did not converge.");
                            break;
                        case Eigen::InvalidInput:
                            log<NUClear::WARN>(
                                "Cholesky decomposition failed. The inputs are invalid, or the algorithm has been "
                                "improperly called. When assertions are enabled, such errors trigger an assert.");
                            break;
                        default: log<NUClear::WARN>("Cholesky decomposition failed. Some other reason."); break;
                    }
                }

                // Detect wether a button has been pressed or not
                detect_button_press(sensors);
            });

        update_loop =
            on<Trigger<RawSensors>,
               Optional<With<Sensors>>,
               With<KinematicsModel>,
               With<Stability>,
               Optional<With<WalkState>>,
               Single,
               Priority::HIGH>()
                .then("Main Sensors Loop",
                      [this](const RawSensors& raw_sensors,
                             const std::shared_ptr<const Sensors>& previous_sensors,
                             const KinematicsModel& kinematics_model,
                             const Stability& stability,
                             const std::shared_ptr<const WalkState>& walk_state) {
                          auto sensors = std::make_unique<Sensors>();

                          // Updates message with raw sensor data
                          update_raw_sensors(sensors, previous_sensors, raw_sensors);

                          // Updates the message with kinematics data
                          update_kinematics(sensors, kinematics_model, raw_sensors);

                          // Updates the Sensors message with odometry data filtered using specified filter
                          switch (cfg.filtering_method.value) {
                              case FilteringMethod::UKF:
                                  update_odometry_ukf(sensors, previous_sensors, raw_sensors);
                                  break;
                              case FilteringMethod::KF:
                                  update_odometry_kf(sensors, previous_sensors, raw_sensors, stability, walk_state);
                                  break;
                              case FilteringMethod::MAHONY:
                                  update_odometry_mahony(sensors, previous_sensors, raw_sensors, stability, walk_state);
                                  break;
                              case FilteringMethod::GROUND_TRUTH:
                                  update_odometry_ground_truth(sensors, raw_sensors);
                                  break;
                              default: log<NUClear::WARN>("Unknown Filtering Method"); break;
                          }

                          // Graph debug information
                          if (log_level <= NUClear::DEBUG) {
                              debug_sensor_filter(sensors, raw_sensors);
                          }

                          emit(std::move(sensors));
                      })
                .disable();
    }

    void SensorFilter::integrate_walkcommand(const double dt, const Stability& stability, const WalkState& walk_state) {
        // Check if we are not currently falling and walking
        if (stability == Stability::DYNAMIC && walk_state.state == WalkState::State::WALKING) {
            // Integrate the walk command to estimate the change in position and yaw orientation
            double dx = walk_state.velocity_target.x() * dt * cfg.deadreckoning_scale.x();
            double dy = walk_state.velocity_target.y() * dt * cfg.deadreckoning_scale.y();
            yaw += walk_state.velocity_target.z() * dt * cfg.deadreckoning_scale.z();
            // Rotate the change in position into world coordinates before adding it to the current position
            Hwt.translation().x() += dx * cos(yaw) - dy * sin(yaw);
            Hwt.translation().y() += dy * cos(yaw) + dx * sin(yaw);
        }
    }

    void SensorFilter::debug_sensor_filter(std::unique_ptr<Sensors>& sensors, const RawSensors& raw_sensors) {
        // Raw accelerometer and gyroscope information
        emit(graph("Gyroscope", sensors->gyroscope.x(), sensors->gyroscope.y(), sensors->gyroscope.z()));
        emit(
            graph("Accelerometer", sensors->accelerometer.x(), sensors->accelerometer.y(), sensors->accelerometer.z()));

        // Foot down sensors state for each foot
        emit(graph(fmt::format("Sensor/Foot Down/{}/Left", std::string(cfg.footDown.method())),
                   sensors->feet[BodySide::LEFT].down));
        emit(graph(fmt::format("Sensor/Foot Down/{}/Right", std::string(cfg.footDown.method())),
                   sensors->feet[BodySide::RIGHT].down));

        // Kinematics information
        const Eigen::Isometry3d Htl(sensors->Htx[ServoID::L_ANKLE_ROLL]);
        const Eigen::Isometry3d Htr(sensors->Htx[ServoID::R_ANKLE_ROLL]);
        Eigen::Matrix<double, 3, 3> Rtl     = Htl.linear();
        Eigen::Matrix<double, 3, 1> Rtl_rpy = MatrixToEulerIntrinsic(Rtl);
        Eigen::Matrix<double, 3, 3> Rtr     = Htr.linear();
        Eigen::Matrix<double, 3, 1> Rtr_rpy = MatrixToEulerIntrinsic(Rtr);
        emit(graph("Left Foot Actual Position", Htl(0, 3), Htl(1, 3), Htl(2, 3)));
        emit(graph("Left Foot Actual Orientation (r,p,y)", Rtl_rpy.x(), Rtl_rpy.y(), Rtl_rpy.z()));
        emit(graph("Right Foot Actual Position", Htr(0, 3), Htr(1, 3), Htr(2, 3)));
        emit(graph("Right Foot Actual Orientation (r,p,y)", Rtr_rpy.x(), Rtr_rpy.y(), Rtr_rpy.z()));
        Eigen::Isometry3d Hpt = Eigen::Isometry3d::Identity();
        if (sensors->feet[BodySide::LEFT].down) {
            Hpt = Htl.inverse();
        }
        else {
            Hpt = Htr.inverse();
        }
        Eigen::Vector3d thetaPT = MatrixToEulerIntrinsic(Hpt.linear());
        emit(graph("Torso actual position (x,y,z)",
                   Hpt.translation().x(),
                   Hpt.translation().y(),
                   Hpt.translation().z()));
        emit(graph("Torso actual orientation (r,p,y)", thetaPT.x(), thetaPT.y(), thetaPT.z()));

        // Odometry information
        Eigen::Isometry3d Hwt    = Eigen::Isometry3d(sensors->Htw).inverse();
        Eigen::Vector3d est_rTWw = Hwt.translation();
        Eigen::Vector3d est_Rwt  = MatrixToEulerIntrinsic(Hwt.rotation());
        emit(graph("Htw est translation (rTWw)", est_rTWw.x(), est_rTWw.y(), est_rTWw.z()));
        emit(graph("Rtw est angles (rpy)", est_Rwt.x(), est_Rwt.y(), est_Rwt.z()));

        // If we have ground truth odometry, then we can debug the error between our estimate and the ground truth
        if (raw_sensors.odometry_ground_truth.exists) {
            Eigen::Isometry3d true_Hwt = Eigen::Isometry3d(raw_sensors.odometry_ground_truth.Htw).inverse();

            // Determine translational distance error
            Eigen::Vector3d true_rTWw  = true_Hwt.translation();
            Eigen::Vector3d error_rTWw = (true_rTWw - est_rTWw).cwiseAbs();
            // Determine yaw, pitch and roll error
            Eigen::Vector3d true_Rwt  = MatrixToEulerIntrinsic(true_Hwt.rotation());
            Eigen::Vector3d error_Rwt = (true_Rwt - est_Rwt).cwiseAbs();
            double quat_rot_error     = Eigen::Quaterniond(true_Hwt.linear() * Hwt.inverse().linear()).w();

            // Graph translation and its error
            emit(graph("Htw true translation (rTWw)", true_rTWw.x(), true_rTWw.y(), true_rTWw.z()));
            emit(graph("Htw translation error", error_rTWw.x(), error_rTWw.y(), error_rTWw.z()));
            // Graph angles and error
            emit(graph("Rwt true angles (rpy)", true_Rwt.x(), true_Rwt.y(), true_Rwt.z()));
            emit(graph("Rwt error (rpy)", error_Rwt.x(), error_Rwt.y(), error_Rwt.z()));
            emit(graph("Quaternion rotational error", quat_rot_error));
        }
    }
}  // namespace module::input
