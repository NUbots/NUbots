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

#include "message/actuation/BodySide.hpp"
#include "message/localisation/Field.hpp"

#include "utility/input/FrameID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::input {

    using message::actuation::BodySide;
    using utility::input::FrameID;
    using utility::input::ServoID;
    using utility::math::euler::EulerIntrinsicToMatrix;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::nusight::graph;
    using utility::support::Expression;

    using message::behaviour::state::Stability;
    using message::localisation::ResetFieldLocalisation;

    using extension::Configuration;

    SensorFilter::SensorFilter(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("SensorFilter.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            // Button config
            cfg.buttons.debounce_threshold = config["buttons"]["debounce_threshold"].as<int>();

            // Foot down config
            const FootDownMethod method = config["foot_down"]["method"].as<std::string>();
            std::map<FootDownMethod, float> thresholds;
            for (const auto& threshold : config["foot_down"]["known_methods"]) {
                thresholds[threshold["name"].as<std::string>()] = threshold["certainty_threshold"].as<float>();
            }
            cfg.footDown.set_method(method, thresholds);

            //  Kinematics Model
            cfg.urdf_path = config["urdf_path"].as<std::string>();
            nugus_model   = tinyrobotics::import_urdf<double, n_joints>(cfg.urdf_path);

            //  Configure Filters
            cfg.filtering_method = config["filtering_method"].as<std::string>();
            configure_ukf(config);
            configure_kf(config);
            configure_mahony(config);

            // Deadreckoning
            Hwp.translation().y() =
                tinyrobotics::forward_kinematics<double, n_joints>(nugus_model,
                                                                   nugus_model.home_configuration(),
                                                                   config["initial_anchor_frame"].as<std::string>())
                    .translation()
                    .y();
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
               Optional<With<Stability>>,
               Optional<With<WalkState>>,
               Single,
               Priority::HIGH>()
                .then("Main Sensors Loop",
                      [this](const RawSensors& raw_sensors,
                             const std::shared_ptr<const Sensors>& previous_sensors,
                             const std::shared_ptr<const Stability>& stability,
                             const std::shared_ptr<const WalkState>& walk_state) {
                          auto sensors = std::make_unique<Sensors>();

                          // Updates message with raw sensor data
                          update_raw_sensors(sensors, previous_sensors, raw_sensors);

                          // Updates the message with kinematics data
                          update_kinematics(sensors, raw_sensors);

                          // Updates the Sensors message with odometry data filtered using specified filter
                          switch (cfg.filtering_method.value) {
                              case FilteringMethod::UKF:
                                  update_odometry_ukf(sensors, previous_sensors, raw_sensors);
                                  break;
                              case FilteringMethod::KF:
                                  update_odometry_kf(sensors, previous_sensors, raw_sensors, stability, walk_state);
                                  break;
                              case FilteringMethod::MAHONY:
                                  update_odometry_mahony(sensors, previous_sensors, raw_sensors, walk_state);
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

        on<Trigger<ResetFieldLocalisation>>().then([this] {
            // Reset the translation and yaw of odometry
            Hwt.translation().x() = 0;
            Hwt.translation().y() = 0;
            yaw                   = 0;
        });
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

    void SensorFilter::anchor_update(std::unique_ptr<Sensors>& sensors, const WalkState& walk_state) {
        // Compute torso pose in world space using kinematics from anchor frame
        if (current_phase.value == WalkState::Phase::LEFT) {
            auto current_translation       = Hwt.translation();
            auto current_rotation          = MatrixToEulerIntrinsic(Hwt.rotation());
            Eigen::Isometry3d measured_Hwt = Hwp * Eigen::Isometry3d(sensors->Htx[FrameID::L_FOOT_BASE].inverse());
            auto measured_translation      = measured_Hwt.translation();
            auto measured_rotation         = MatrixToEulerIntrinsic(measured_Hwt.rotation());
            // Apply exponential filter to the translation and rotation
            double alpha      = 0.9;
            Hwt.translation() = alpha * measured_translation + (1 - alpha) * current_translation;
            Hwt.linear()      = EulerIntrinsicToMatrix(alpha * measured_rotation + (1 - alpha) * current_rotation);
        }
        else if (current_phase.value == WalkState::Phase::RIGHT) {
            auto current_translation       = Hwt.translation();
            auto current_rotation          = MatrixToEulerIntrinsic(Hwt.rotation());
            Eigen::Isometry3d measured_Hwt = Hwp * Eigen::Isometry3d(sensors->Htx[FrameID::R_FOOT_BASE].inverse());
            auto measured_translation      = measured_Hwt.translation();
            auto measured_rotation         = MatrixToEulerIntrinsic(measured_Hwt.rotation());
            // Apply exponential filter to the translation and rotation
            double alpha      = 0.9;
            Hwt.translation() = alpha * measured_translation + (1 - alpha) * current_translation;
            Hwt.linear()      = EulerIntrinsicToMatrix(alpha * measured_rotation + (1 - alpha) * current_rotation);
        }


        // Update the anchor {a} frame if a support phase switch just occurred (could be done with foot down sensors)
        if (walk_state.phase != current_phase) {
            current_phase = walk_state.phase;
            if (current_phase.value == WalkState::Phase::LEFT) {
                // Update the anchor frame to the base of left foot
                Hwp = Hwt * sensors->Htx[FrameID::L_FOOT_BASE];
            }
            else if (current_phase.value == WalkState::Phase::RIGHT) {
                // Update the anchor frame to the base of right foot
                Hwp = Hwt * sensors->Htx[FrameID::R_FOOT_BASE];
            }
        }
        // Set the z translation, roll and pitch of the anchor frame to 0 as assumed to be on flat ground
        Hwp.translation().z() = 0;
        Hwp.linear() =
            Eigen::AngleAxisd(MatrixToEulerIntrinsic(Hwp.linear()).z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        sensors->Hwp = Hwp;
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
        const Eigen::Isometry3d Htl(sensors->Htx[FrameID::L_ANKLE_ROLL]);
        const Eigen::Isometry3d Htr(sensors->Htx[FrameID::R_ANKLE_ROLL]);
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
        emit(graph("Hwt est translation (rTWw)", est_rTWw.x(), est_rTWw.y(), est_rTWw.z()));
        emit(graph("Rwt est angles (rpy)", est_Rwt.x(), est_Rwt.y(), est_Rwt.z()));

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
            emit(graph("Hwt true translation (rTWw)", true_rTWw.x(), true_rTWw.y(), true_rTWw.z()));
            emit(graph("Hwt translation error", error_rTWw.x(), error_rTWw.y(), error_rTWw.z()));
            // Graph angles and error
            emit(graph("Rwt true angles (rpy)", true_Rwt.x(), true_Rwt.y(), true_Rwt.z()));
            emit(graph("Rwt error (rpy)", error_Rwt.x(), error_Rwt.y(), error_Rwt.z()));
            emit(graph("Quaternion rotational error", quat_rot_error));
        }
    }
}  // namespace module::input
