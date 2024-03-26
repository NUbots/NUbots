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

namespace module::input {

    using extension::Configuration;

    SensorFilter::SensorFilter(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("SensorFilter.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            // Button config
            cfg.button_debounce_threshold = config["buttons"]["debounce_threshold"].as<int>();

            // Foot down config
            const FootDownMethod method = config["foot_down"]["method"].as<std::string>();
            std::map<FootDownMethod, float> thresholds;
            for (const auto& threshold : config["foot_down"]["known_methods"]) {
                thresholds[threshold["name"].as<std::string>()] = threshold["certainty_threshold"].as<float>();
            }
            cfg.foot_down.set_method(method, thresholds);

            // Import URDF model
            nugus_model = tinyrobotics::import_urdf<double, n_servos>(config["urdf_path"].as<std::string>());

            // Configure the Mahony filter
            cfg.Ki           = config["mahony"]["Ki"].as<Expression>();
            cfg.Kp           = config["mahony"]["Kp"].as<Expression>();
            cfg.initial_bias = Eigen::Vector3d(config["mahony"]["initial_bias"].as<Expression>());
            cfg.initial_Hwt.linear() =
                rpy_intrinsic_to_mat(Eigen::Vector3d(config["mahony"]["initial_rpy"].as<Expression>()));
            bias_mahony = cfg.initial_bias;
            Hwt_mahony  = cfg.initial_Hwt;

            // Configure the UKF
            // Set our measurement noise
            cfg.ukf.noise.measurement.flat_foot_translation =
                Eigen::Vector3d(config["ukf"]["noise"]["measurement"]["flat_foot_translation"].as<Expression>())
                    .asDiagonal();

            // Set our process noises
            cfg.ukf.noise.process.position = config["ukf"]["noise"]["process"]["position"].as<Expression>();
            cfg.ukf.noise.process.velocity = config["ukf"]["noise"]["process"]["velocity"].as<Expression>();

            // Set our motion model's process noise
            MotionModel<double>::StateVec process_noise;
            process_noise.rTWw      = cfg.ukf.noise.process.position;
            process_noise.vTw       = cfg.ukf.noise.process.velocity;
            ukf.model.process_noise = process_noise;

            // Set our initial means
            cfg.ukf.initial.mean.position = config["ukf"]["initial"]["mean"]["position"].as<Expression>();
            cfg.ukf.initial.mean.velocity = config["ukf"]["initial"]["mean"]["velocity"].as<Expression>();

            // Set out initial covariance
            cfg.ukf.initial.covariance.position = config["ukf"]["initial"]["covariance"]["position"].as<Expression>();
            cfg.ukf.initial.covariance.velocity = config["ukf"]["initial"]["covariance"]["velocity"].as<Expression>();

            // Set our initial state with the config means and covariances, flagging the filter to reset it
            cfg.initial_mean.rTWw = cfg.ukf.initial.mean.position;
            cfg.initial_mean.vTw  = cfg.ukf.initial.mean.velocity;

            cfg.initial_covariance.rTWw = cfg.ukf.initial.covariance.position;
            cfg.initial_covariance.vTw  = cfg.ukf.initial.covariance.velocity;
            ukf.set_state(cfg.initial_mean.getStateVec(), cfg.initial_covariance.asDiagonal());

            // Initialise the anchor frame (left foot base)
            Hwa.translation().y() = forward_kinematics<double, n_servos>(nugus_model,
                                                                         nugus_model.home_configuration(),
                                                                         std::string("left_foot_base"))
                                        .translation()
                                        .y();

            cfg.use_ground_truth = config["use_ground_truth"].as<bool>();
        });

        on<Trigger<RawSensors>, Optional<With<Sensors>>, Optional<With<WalkState>>, Single, Priority::HIGH>().then(
            "Main Sensors Loop",
            [this](const RawSensors& raw_sensors,
                   const std::shared_ptr<const Sensors>& previous_sensors,
                   const std::shared_ptr<const WalkState>& walk_state) {
                auto sensors = std::make_unique<Sensors>();

                // Raw sensors (Accelerometer, Gyroscope, etc.)
                update_raw_sensors(sensors, previous_sensors, raw_sensors);

                // Kinematics (Htw, foot down, CoM, etc.)
                update_kinematics(sensors, raw_sensors);

                // Odometry (Htw and Hrw)
                update_odometry(sensors, previous_sensors, raw_sensors, walk_state);

                // Graph debug information
                if (log_level <= NUClear::DEBUG) {
                    debug_sensor_filter(sensors, raw_sensors);
                }

                emit(std::move(sensors));
            });

        on<Last<20, Trigger<RawSensors>>, Single>().then(
            [this](const std::list<std::shared_ptr<const RawSensors>>& raw_sensors) {
                // Detect wether a button has been pressed or not in the last 20 messages
                detect_button_press(raw_sensors);
            });

        on<Trigger<ResetFieldLocalisation>>().then([this] {
            // Reset the mahony filter
            bias_mahony = cfg.initial_bias;
            Hwt_mahony  = cfg.initial_Hwt;

            // Reset anchor frame
            Hwa                   = Eigen::Isometry3d::Identity();
            Hwa.translation().y() = tinyrobotics::forward_kinematics<double, n_servos>(nugus_model,
                                                                                       nugus_model.home_configuration(),
                                                                                       std::string("left_foot_base"))
                                        .translation()
                                        .y();
        });
    }

    void SensorFilter::update_kinematics(std::unique_ptr<Sensors>& sensors, const RawSensors& raw_sensors) {
        // Convert the sensor joint angles to a configuration vector
        Eigen::Matrix<double, n_servos, 1> q = sensors_to_configuration<double, n_servos>(sensors);

        // **************** Kinematics ****************
        // Htx is a map from FrameID to homogeneous transforms from each frame to the torso
        std::vector<Eigen::Isometry3d> fk = tinyrobotics::forward_kinematics(nugus_model, q);
        auto Htx                          = forward_kinematics_to_servo_map(fk);
        for (const auto& entry : Htx) {
            sensors->Htx[entry.first] = entry.second.matrix();
        }

        // **************** Centre of Mass  ****************
        sensors->rMTt = tinyrobotics::center_of_mass(nugus_model, q);

        // **************** Foot Down Information ****************
        sensors->feet[BodySide::RIGHT].down = true;
        sensors->feet[BodySide::LEFT].down  = true;
        const Eigen::Isometry3d Htr(sensors->Htx[FrameID::R_ANKLE_ROLL]);
        const Eigen::Isometry3d Htl(sensors->Htx[FrameID::L_ANKLE_ROLL]);
        const Eigen::Isometry3d Hlr = Htl.inverse() * Htr;
        const Eigen::Vector3d rRLl  = Hlr.translation();
        switch (cfg.foot_down.method()) {
            case FootDownMethod::Z_HEIGHT:
                if (rRLl.z() < -cfg.foot_down.threshold()) {
                    sensors->feet[BodySide::LEFT].down = false;
                }
                else if (rRLl.z() > cfg.foot_down.threshold()) {
                    sensors->feet[BodySide::RIGHT].down = false;
                }
                break;
            case FootDownMethod::FSR:
                // Determine if any two diagonally opposite FSRs are in contact with the ground, which is either fsr1
                // and fsr3, or fsr2 and fsr4. A FSR is in contact with the ground if its value is greater than the
                // certainty threshold
                sensors->feet[BodySide::LEFT].down = (raw_sensors.fsr.left.fsr1 > cfg.foot_down.threshold()
                                                      && raw_sensors.fsr.left.fsr3 > cfg.foot_down.threshold())
                                                     || (raw_sensors.fsr.left.fsr2 > cfg.foot_down.threshold()
                                                         && raw_sensors.fsr.left.fsr4 > cfg.foot_down.threshold());
                sensors->feet[BodySide::RIGHT].down = (raw_sensors.fsr.right.fsr1 > cfg.foot_down.threshold()
                                                       && raw_sensors.fsr.right.fsr3 > cfg.foot_down.threshold())
                                                      || (raw_sensors.fsr.right.fsr2 > cfg.foot_down.threshold()
                                                          && raw_sensors.fsr.right.fsr4 > cfg.foot_down.threshold());
                break;
            default: log<NUClear::WARN>("Unknown foot down method"); break;
        }
    }

    void SensorFilter::update_raw_sensors(std::unique_ptr<Sensors>& sensors,
                                          const std::shared_ptr<const Sensors>& previous_sensors,
                                          const RawSensors& raw_sensors) {

        // Mask to ignore the alert bit (because servo errors are handled separately)
        bool subcontroller_packet_error =
            (raw_sensors.subcontroller_error & ~RawSensors::PacketError::ALERT) != RawSensors::PacketError::PACKET_OK;

        // Check for errors on the platform and FSRs
        if (subcontroller_packet_error) {
            NUClear::log<NUClear::WARN>(make_packet_error_string("Platform", raw_sensors.subcontroller_error));
        }

        // **************** Servos ****************
        for (uint32_t id = 0; id < n_servos; ++id) {
            const auto& raw_servo       = getRawServo(id, raw_sensors);
            const auto& hardware_status = raw_servo.hardware_error;

            // Check for an error on the servo and report it
            if (hardware_status != RawSensors::HardwareError::HARDWARE_OK) {
                NUClear::log<NUClear::WARN>(make_servo_hardware_error_string(raw_servo, id));
            }

            // If the RawSensors message for this servo has an error, but we have a previous Sensors message available,
            // then for some fields we will want to selectively use the old Sensors value, otherwise we just use the new
            // values as is
            sensors->servo.emplace_back(
                hardware_status,
                id,
                raw_servo.torque_enabled,
                raw_servo.position_p_gain,
                raw_servo.position_i_gain,
                raw_servo.position_d_gain,
                raw_servo.goal_position,
                raw_servo.profile_velocity,
                /* If there is an encoder error, then use the last known good position */
                ((hardware_status == RawSensors::HardwareError::MOTOR_ENCODER) && previous_sensors)
                    ? previous_sensors->servo[id].present_position
                    : raw_servo.present_position,
                /* If there is an encoder error, then use the last known good velocity */
                ((hardware_status == RawSensors::HardwareError::MOTOR_ENCODER) && previous_sensors)
                    ? previous_sensors->servo[id].present_velocity
                    : raw_servo.present_velocity,
                /* We may get a RawSensors::HardwareError::OVERLOAD error, but in this case the load value isn't *wrong*
                   so it doesn't make sense to use the last "good" value */
                raw_servo.present_current,
                /* Similarly for a RawSensors::HardwareError::INPUT_VOLTAGE error, no special action is needed */
                raw_servo.voltage,
                /* And similarly here for a RawSensors::HardwareError::OVERHEATING error, we still pass the temperature
                   no matter what */
                static_cast<float>(raw_servo.temperature));
        }

        // **************** Accelerometer and Gyroscope ****************
        // If we have a previous Sensors and our platform has errors then reuse our last sensor value of the
        // accelerometer
        sensors->accelerometer =
            subcontroller_packet_error ? previous_sensors->accelerometer : raw_sensors.accelerometer.cast<double>();
        sensors->gyroscope =
            subcontroller_packet_error ? previous_sensors->gyroscope : raw_sensors.gyroscope.cast<double>();

        // If we have a previous Sensors message AND (our platform has errors OR the gyro is spinning too fast) then
        // reuse our last sensor value of the gyroscope. Note: One of the gyros would occasionally
        // throw massive numbers without an error flag and if our hardware is working as intended, it should never
        // read that we're spinning at 2 revs/s
        if (raw_sensors.gyroscope.norm() > 4.0 * M_PI) {
            NUClear::log<NUClear::WARN>("Bad gyroscope value", raw_sensors.gyroscope.norm());
            if (previous_sensors) {
                sensors->gyroscope = previous_sensors->gyroscope;
            }
        }

        // **************** Timestamp ****************
        sensors->timestamp = raw_sensors.timestamp;

        // **************** Battery Voltage  ****************
        // Update the current battery voltage of the whole robot
        sensors->voltage = raw_sensors.battery;

        // **************** Buttons and LEDs ****************
        sensors->button.reserve(2);
        sensors->button.emplace_back(0, raw_sensors.buttons.left);
        sensors->button.emplace_back(1, raw_sensors.buttons.middle);
        sensors->led.reserve(5);
        sensors->led.emplace_back(0, raw_sensors.led_panel.led2 ? 0xFF0000 : 0);
        sensors->led.emplace_back(1, raw_sensors.led_panel.led3 ? 0xFF0000 : 0);
        sensors->led.emplace_back(2, raw_sensors.led_panel.led4 ? 0xFF0000 : 0);
        sensors->led.emplace_back(3, raw_sensors.head_led.RGB);  // Head
        sensors->led.emplace_back(4, raw_sensors.eye_led.RGB);   // Eye
    }

    void SensorFilter::detect_button_press(const std::list<std::shared_ptr<const RawSensors>>& sensors) {
        int left_count   = 0;
        int middle_count = 0;
        // If we have any downs in the last 20 frames then we are button pushed
        for (const auto& s : sensors) {
            if (s->buttons.left && (s->subcontroller_error == 0u)) {
                ++left_count;
            }
            if (s->buttons.middle && (s->subcontroller_error == 0u)) {
                ++middle_count;
            }
        }
        bool new_left_down   = left_count > cfg.button_debounce_threshold;
        bool new_middle_down = middle_count > cfg.button_debounce_threshold;
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
    }

    void SensorFilter::update_odometry(std::unique_ptr<Sensors>& sensors,
                                       const std::shared_ptr<const Sensors>& previous_sensors,
                                       const RawSensors& raw_sensors,
                                       const std::shared_ptr<const WalkState>& walk_state) {


        if (!cfg.use_ground_truth) {
            // Compute time since last update
            const double dt = std::max(
                std::chrono::duration_cast<std::chrono::duration<double>>(
                    raw_sensors.timestamp - (previous_sensors ? previous_sensors->timestamp : raw_sensors.timestamp))
                    .count(),
                0.0);

            // Update the current support phase is not the same as the walk state, a support phase change has occurred
            if (current_support_phase != walk_state->support_phase && walk_state != nullptr) {
                // Update the current support phase to the new support phase
                current_support_phase = walk_state->support_phase;

                // Compute the new anchor frame (Hwa) (new support foot)
                if (current_support_phase.value == WalkState::SupportPhase::LEFT) {
                    auto Hrl = sensors->Htx[FrameID::R_FOOT_BASE].inverse() * sensors->Htx[FrameID::L_FOOT_BASE];
                    Hwa      = Hwa * Hrl;
                }
                else if (current_support_phase.value == WalkState::SupportPhase::RIGHT) {
                    auto Hlr = sensors->Htx[FrameID::L_FOOT_BASE].inverse() * sensors->Htx[FrameID::R_FOOT_BASE];
                    Hwa      = Hwa * Hlr;
                }

                // Set the z translation, roll and pitch of the anchor frame to 0 as assumed to be on field plane
                Hwa.translation().z() = 0;
                Hwa.linear() = rpy_intrinsic_to_mat(Eigen::Vector3d(0, 0, mat_to_rpy_intrinsic(Hwa.linear()).z()));
            }

            // Compute torso pose using kinematics from anchor frame (current support foot)
            if (current_support_phase.value == WalkState::SupportPhase::LEFT) {
                Hat = sensors->Htx[FrameID::L_FOOT_BASE].inverse();
            }
            else if (current_support_phase.value == WalkState::SupportPhase::RIGHT) {
                Hat = sensors->Htx[FrameID::R_FOOT_BASE].inverse();
            }

            // Perform Anchor Update (x, y, z, yaw)
            Hwt_anchor                 = Hwa * Hat;
            Eigen::Vector3d rpy_anchor = mat_to_rpy_intrinsic(Hwt_anchor.linear());

            // Perform Mahony update (roll, pitch)
            mahony_update(sensors->accelerometer, sensors->gyroscope, Hwt_mahony, dt, cfg.Ki, cfg.Kp, bias_mahony);

            // Convert the rotation matrices from anchor and mahony method into euler angles
            Eigen::Vector3d rpy_mahony = mat_to_rpy_intrinsic(Hwt_mahony.linear());
            // Remove yaw from mahony filter (prevents it breaking after numerous rotations)
            Hwt_mahony.linear() = rpy_intrinsic_to_mat(Eigen::Vector3d(rpy_mahony.x(), rpy_mahony.y(), 0.0));
            // Construct world {w} to torso {t} space transform
            Eigen::Isometry3d Hwt = Eigen::Isometry3d::Identity();
            // Take the translation from the anchor method
            Hwt.translation() = Hwt_anchor.translation();
            // Fuse roll + pitch of mahony filter with yaw of anchor method
            Hwt.linear() = rpy_intrinsic_to_mat(Eigen::Vector3d(rpy_mahony.x(), rpy_mahony.y(), rpy_anchor.z()));
            sensors->Htw = Hwt.inverse();

            // Construct robot {r} to world {w} space transform (just x-y translation and yaw rotation)
            Eigen::Isometry3d Hwr = Eigen::Isometry3d::Identity();
            Hwr.linear()          = Eigen::AngleAxisd(rpy_anchor.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
            Hwr.translation()     = Eigen::Vector3d(Hwt_anchor.translation().x(), Hwt_anchor.translation().y(), 0.0);
            sensors->Hrw          = Hwr.inverse();

            // UKF update for velocity
            Eigen::Vector3d rTWw = Hwt.translation();
            ukf.measure(rTWw,
                        cfg.ukf.noise.measurement.flat_foot_translation,
                        MeasurementType::FLAT_FOOT_TRANSLATION());
            ukf.time(dt);
            auto state   = MotionModel<double>::StateVec(ukf.get_state());
            sensors->vTw = state.vTw;
        }
        else {
            // Construct world {w} to torso {t} space transform from ground truth
            Eigen::Isometry3d Hwt = Eigen::Isometry3d(raw_sensors.odometry_ground_truth.Htw).inverse();
            sensors->Htw          = Hwt.inverse();
            // Construct robot {r} to world {w} space transform from ground truth
            Eigen::Isometry3d Hwr = Eigen::Isometry3d::Identity();
            Hwr.linear() =
                Eigen::AngleAxisd(mat_to_rpy_intrinsic(Hwt.linear()).z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
            Hwr.translation() = Eigen::Vector3d(Hwt.translation().x(), Hwt.translation().y(), 0.0);
            sensors->Hrw      = Hwr.inverse();
            sensors->vTw      = raw_sensors.odometry_ground_truth.vTw;
        }
    }

    void SensorFilter::debug_sensor_filter(std::unique_ptr<Sensors>& sensors, const RawSensors& raw_sensors) {
        // Raw accelerometer and gyroscope information
        emit(graph("Gyroscope", sensors->gyroscope.x(), sensors->gyroscope.y(), sensors->gyroscope.z()));
        emit(
            graph("Accelerometer", sensors->accelerometer.x(), sensors->accelerometer.y(), sensors->accelerometer.z()));

        // Foot down sensors state for each foot
        emit(graph(fmt::format("Foot Down/{}/Left", std::string(cfg.foot_down.method())),
                   sensors->feet[BodySide::LEFT].down));
        emit(graph(fmt::format("Foot Down/{}/Right", std::string(cfg.foot_down.method())),
                   sensors->feet[BodySide::RIGHT].down));

        // Odometry information
        Eigen::Isometry3d Hwt    = Eigen::Isometry3d(sensors->Htw).inverse();
        Eigen::Vector3d est_rTWw = Hwt.translation();
        Eigen::Vector3d est_Rwt  = mat_to_rpy_intrinsic(Hwt.rotation());
        emit(graph("Hwt est translation (rTWw)", est_rTWw.x(), est_rTWw.y(), est_rTWw.z()));
        emit(graph("Rwt est angles (rpy)", est_Rwt.x(), est_Rwt.y(), est_Rwt.z()));
        emit(graph("vTw est", sensors->vTw.x(), sensors->vTw.y(), sensors->vTw.z()));

        // If we have ground truth odometry, then we can debug the error between our estimate and the ground truth
        if (raw_sensors.odometry_ground_truth.exists) {
            Eigen::Isometry3d true_Hwt = Eigen::Isometry3d(raw_sensors.odometry_ground_truth.Htw).inverse();

            // Determine translation and orientation error
            Eigen::Vector3d true_rTWw  = true_Hwt.translation();
            Eigen::Vector3d error_rTWw = (true_rTWw - est_rTWw).cwiseAbs();
            Eigen::Vector3d true_Rwt   = mat_to_rpy_intrinsic(true_Hwt.rotation());
            Eigen::Vector3d error_Rwt  = (true_Rwt - est_Rwt).cwiseAbs();
            double quat_rot_error      = Eigen::Quaterniond(true_Hwt.linear() * Hwt.inverse().linear()).w();
            Eigen::Vector3d true_vTw   = Eigen::Vector3d(raw_sensors.odometry_ground_truth.vTw);
            Eigen::Vector3d error_vTw  = (true_vTw - sensors->vTw).cwiseAbs();

            // Graph translation, angles and error
            emit(graph("Hwt true translation (rTWw)", true_rTWw.x(), true_rTWw.y(), true_rTWw.z()));
            emit(graph("Hwt translation error", error_rTWw.x(), error_rTWw.y(), error_rTWw.z()));
            emit(graph("Rwt true angles (rpy)", true_Rwt.x(), true_Rwt.y(), true_Rwt.z()));
            emit(graph("Rwt error (rpy)", error_Rwt.x(), error_Rwt.y(), error_Rwt.z()));
            emit(graph("Quaternion rotational error", quat_rot_error));
            emit(graph("vTw true", true_vTw.x(), true_vTw.y(), true_vTw.z()));
            emit(graph("vTw error", error_vTw.x(), error_vTw.y(), error_vTw.z()));
        }
    }

}  // namespace module::input
