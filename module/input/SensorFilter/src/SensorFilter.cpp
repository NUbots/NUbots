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

#include "SensorFilter.hpp"

#include "Kinematics.hpp"
#include "OdometryKF.hpp"
#include "OdometryUKF.hpp"
#include "RawSensors.hpp"

#include "extension/Configuration.hpp"

namespace module::input {

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

            // Motion filter config
            // Set velocity decay
            cfg.motionFilter.velocity_decay = config["motion_filter"]["update"]["velocity_decay"].as<Expression>();
            motionFilter.model.timeUpdateVelocityDecay = cfg.motionFilter.velocity_decay;

            // Set our measurement noises
            cfg.motionFilter.noise.measurement.accelerometer =
                Eigen::Vector3d(config["motion_filter"]["noise"]["measurement"]["accelerometer"].as<Expression>())
                    .asDiagonal();
            cfg.motionFilter.noise.measurement.accelerometer_magnitude =
                Eigen::Vector3d(
                    config["motion_filter"]["noise"]["measurement"]["accelerometer_magnitude"].as<Expression>())
                    .asDiagonal();
            cfg.motionFilter.noise.measurement.gyroscope =
                Eigen::Vector3d(config["motion_filter"]["noise"]["measurement"]["gyroscope"].as<Expression>())
                    .asDiagonal();
            cfg.motionFilter.noise.measurement.flat_foot_odometry =
                Eigen::Vector3d(config["motion_filter"]["noise"]["measurement"]["flat_foot_odometry"].as<Expression>())
                    .asDiagonal();
            cfg.motionFilter.noise.measurement.flat_foot_orientation =
                Eigen::Vector4d(
                    config["motion_filter"]["noise"]["measurement"]["flat_foot_orientation"].as<Expression>())
                    .asDiagonal();

            // Set our process noises
            cfg.motionFilter.noise.process.position =
                config["motion_filter"]["noise"]["process"]["position"].as<Expression>();
            cfg.motionFilter.noise.process.velocity =
                config["motion_filter"]["noise"]["process"]["velocity"].as<Expression>();
            cfg.motionFilter.noise.process.rotation =
                config["motion_filter"]["noise"]["process"]["rotation"].as<Expression>();
            cfg.motionFilter.noise.process.rotational_velocity =
                config["motion_filter"]["noise"]["process"]["rotational_velocity"].as<Expression>();

            // Set our motion model's process noise
            MotionModel<double>::StateVec process_noise;
            process_noise.rTWw               = cfg.motionFilter.noise.process.position;
            process_noise.vTw                = cfg.motionFilter.noise.process.velocity;
            process_noise.Rwt                = cfg.motionFilter.noise.process.rotation;
            process_noise.omegaTTt           = cfg.motionFilter.noise.process.rotational_velocity;
            motionFilter.model.process_noise = process_noise;

            // Set our initial means
            cfg.motionFilter.initial.mean.position =
                config["motion_filter"]["initial"]["mean"]["position"].as<Expression>();
            cfg.motionFilter.initial.mean.velocity =
                config["motion_filter"]["initial"]["mean"]["velocity"].as<Expression>();
            cfg.motionFilter.initial.mean.rotation =
                config["motion_filter"]["initial"]["mean"]["rotation"].as<Expression>();
            cfg.motionFilter.initial.mean.rotational_velocity =
                config["motion_filter"]["initial"]["mean"]["rotational_velocity"].as<Expression>();

            // Set out initial covariance
            cfg.motionFilter.initial.covariance.position =
                config["motion_filter"]["initial"]["covariance"]["position"].as<Expression>();
            cfg.motionFilter.initial.covariance.velocity =
                config["motion_filter"]["initial"]["covariance"]["velocity"].as<Expression>();
            cfg.motionFilter.initial.covariance.rotation =
                config["motion_filter"]["initial"]["covariance"]["rotation"].as<Expression>();
            cfg.motionFilter.initial.covariance.rotational_velocity =
                config["motion_filter"]["initial"]["covariance"]["rotational_velocity"].as<Expression>();

            // Set our initial state with the config means and covariances, flagging the filter to reset it
            cfg.initial_mean.rTWw     = cfg.motionFilter.initial.mean.position;
            cfg.initial_mean.vTw      = cfg.motionFilter.initial.mean.velocity;
            cfg.initial_mean.Rwt      = cfg.motionFilter.initial.mean.rotation;
            cfg.initial_mean.omegaTTt = cfg.motionFilter.initial.mean.rotational_velocity;

            cfg.initial_covariance.rTWw     = cfg.motionFilter.initial.covariance.position;
            cfg.initial_covariance.vTw      = cfg.motionFilter.initial.covariance.velocity;
            cfg.initial_covariance.Rwt      = cfg.motionFilter.initial.covariance.rotation;
            cfg.initial_covariance.omegaTTt = cfg.motionFilter.initial.covariance.rotational_velocity;
            motionFilter.set_state(cfg.initial_mean.getStateVec(), cfg.initial_covariance.asDiagonal());

            // Don't filter any sensors until we have initialised the filter
            update_loop.disable();
            reset_filter.store(true);

            // Kalman Filter Model Matrix's
            Eigen::Matrix<double, n_states, n_states> Ac =
                Eigen::Matrix<double, n_states, n_states>(config["kalman_filter"]["Ac"].as<Expression>());
            Eigen::Matrix<double, n_inputs, n_inputs> Bc;
            Eigen::Matrix<double, n_measurements, n_states> C =
                Eigen::Matrix<double, n_measurements, n_states>(config["kalman_filter"]["C"].as<Expression>());
            Eigen::Matrix<double, n_states, n_states> Q;
            Q.diagonal() = Eigen::VectorXd(config["kalman_filter"]["Q"].as<Expression>());

            Eigen::Matrix<double, n_measurements, n_measurements> R;
            R.diagonal() = Eigen::VectorXd(config["kalman_filter"]["R"].as<Expression>());

            pose_filter.update(Ac, Bc, C, Q, R);
            pose_filter.reset(Eigen::VectorXd::Zero(n_states), Eigen::MatrixXd::Identity(n_states, n_states));

            // Deadreckoning tuning parameters
            cfg.deadreckoning_scale_dx     = config["deadreckoning_scale_dx"].as<Expression>();
            cfg.deadreckoning_scale_dy     = config["deadreckoning_scale_dy"].as<Expression>();
            cfg.deadreckoning_scale_dtheta = config["deadreckoning_scale_dtheta"].as<Expression>();
        });

        on<Last<20, Trigger<RawSensors>>, Single>().then(
            [this](const std::list<std::shared_ptr<const RawSensors>>& sensors) {
                // If we need to reset the filter, do that here
                if (reset_filter.load()) {
                    // We have finished resetting the filter now
                    switch (motionFilter.reset(cfg.initial_mean.getStateVec(), cfg.initial_covariance.asDiagonal())) {
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
                int left_count   = 0;
                int middle_count = 0;
                // If we have any downs in the last 20 frames then we are button pushed
                for (const auto& s : sensors) {
                    if (s->buttons.left && (s->platform_error_flags == 0u)) {
                        ++left_count;
                    }
                    if (s->buttons.middle && (s->platform_error_flags == 0u)) {
                        ++middle_count;
                    }
                }
                bool new_left_down   = left_count > cfg.buttons.debounce_threshold;
                bool new_middle_down = middle_count > cfg.buttons.debounce_threshold;
                if (new_left_down != left_down) {
                    left_down = new_left_down;
                    if (new_left_down) {
                        log<NUClear::INFO>("Left Button Down");
                        emit(std::make_unique<ButtonLeftDown>());
                    }
                    else {
                        log<NUClear::INFO>("Left Button Up");
                        emit(std::make_unique<ButtonLeftUp>());
                    }
                }
                if (new_middle_down != middle_down) {
                    middle_down = new_middle_down;
                    if (new_middle_down) {
                        log<NUClear::INFO>("Middle Button Down");
                        emit(std::make_unique<ButtonMiddleDown>());
                    }
                    else {
                        log<NUClear::INFO>("Middle Button Up");
                        emit(std::make_unique<ButtonMiddleUp>());
                    }
                }
            });

        on<Trigger<WalkCommand>>().then([this](const WalkCommand& wc) {
            if (!walk_engine_enabled) {
                walk_engine_enabled = true;
            }
            walk_command = wc.command;
        });

        on<Trigger<EnableWalkEngineCommand>>().then([this]() { walk_engine_enabled = true; });

        on<Trigger<DisableWalkEngineCommand>>().then([this]() {
            walk_command        = Eigen::Vector3d::Zero();
            walk_engine_enabled = false;
        });

        on<Trigger<StopCommand>>().then([this]() {
            walk_command        = Eigen::Vector3d::Zero();
            walk_engine_enabled = false;
        });

        on<Trigger<ExecuteGetup>>().then([this]() { falling = true; });

        on<Trigger<KillGetup>>().then([this]() { falling = false; });

        update_loop = on<Trigger<RawSensors>, Optional<With<Sensors>>, With<KinematicsModel>, Single, Priority::HIGH>()
                          .then("Main Sensors Loop",
                                [this](const RawSensors& raw_sensors,
                                       const std::shared_ptr<const Sensors>& previous_sensors,
                                       const KinematicsModel& kinematics_model) {
                                    auto sensors = std::make_unique<Sensors>();

                                    // Updates the Sensors message with raw sensor data, including the timestamp,
                                    // battery voltage, servo sensors, accelerometer, gyroscope, buttons, and LED.
                                    update_raw_sensors(sensors, previous_sensors, raw_sensors);

                                    // Updates the Sensors message with kinematic data, including the homogeneous
                                    // transforms from each servo to the torso, the centre of mass, the inertia
                                    // tensor, and the contact state of each foot
                                    update_kinematics(sensors, kinematics_model, raw_sensors);

                                    // Updates the Sensors message with odometry data filtered using UKF. This includes
                                    // the position, orientation, velocity and rotational velocity of the torso in world
                                    // space.
                                    update_odometry_kf(sensors, previous_sensors, raw_sensors);

                                    // Graph debug information
                                    if (log_level <= NUClear::DEBUG) {
                                        debug_sensor_filter(sensors, raw_sensors);
                                    }

                                    emit(std::move(sensors));
                                })
                          .disable();
    }

    void SensorFilter::debug_sensor_filter(std::unique_ptr<Sensors>& sensors, const RawSensors& raw_sensors) {
        // Graph the raw accelerometer and gyroscope data
        emit(graph("Gyroscope", sensors->gyroscope.x(), sensors->gyroscope.y(), sensors->gyroscope.z()));
        emit(
            graph("Accelerometer", sensors->accelerometer.x(), sensors->accelerometer.y(), sensors->accelerometer.z()));

        // Graph the foot down sensors state for each foot
        emit(graph(fmt::format("Sensor/Foot Down/{}/Left", std::string(cfg.footDown.method())),
                   sensors->feet[BodySide::LEFT].down));
        emit(graph(fmt::format("Sensor/Foot Down/{}/Right", std::string(cfg.footDown.method())),
                   sensors->feet[BodySide::RIGHT].down));

        // Graph kinematics information
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

        // Graph odometry information
        Eigen::Isometry3d Hwt = Eigen::Isometry3d(sensors->Htw).inverse();

        // Translation
        Eigen::Vector3d est_rTWw = Hwt.translation();
        emit(graph("Htw est translation (rTWw)", est_rTWw.x(), est_rTWw.y(), est_rTWw.z()));

        // Orientation
        Eigen::Vector3d est_Rwt = MatrixToEulerIntrinsic(Hwt.rotation());
        emit(graph("Rtw est angles (rpy)", est_Rwt.x(), est_Rwt.y(), est_Rwt.z()));

        // If we have ground truth odometry, then we can debug the error between our estimate and the ground truth
        if (raw_sensors.odometry_ground_truth.exists) {
            Eigen::Isometry3d true_Hwt = Eigen::Isometry3d(raw_sensors.odometry_ground_truth.Htw).inverse();

            // Determine translational distance error
            Eigen::Vector3d true_rTWw  = true_Hwt.translation();
            Eigen::Vector3d error_rTWw = (true_rTWw - est_rTWw).cwiseAbs();

            // Graph translation and its error
            emit(graph("Htw true translation (rTWw)", true_rTWw.x(), true_rTWw.y(), true_rTWw.z()));
            emit(graph("Htw translation error", error_rTWw.x(), error_rTWw.y(), error_rTWw.z()));

            // Determine yaw, pitch and roll error
            Eigen::Vector3d true_Rwt  = MatrixToEulerIntrinsic(true_Hwt.rotation());
            Eigen::Vector3d error_Rwt = (true_Rwt - est_Rwt).cwiseAbs();
            double quat_rot_error     = Eigen::Quaterniond(true_Hwt.linear() * Hwt.inverse().linear()).w();

            // Graph angles and error
            emit(graph("Rwt true angles (rpy)", true_Rwt.x(), true_Rwt.y(), true_Rwt.z()));
            emit(graph("Rwt error (rpy)", error_Rwt.x(), error_Rwt.y(), error_Rwt.z()));
            emit(graph("Quaternion rotational error", quat_rot_error));
        }
    }
}  // namespace module::input
