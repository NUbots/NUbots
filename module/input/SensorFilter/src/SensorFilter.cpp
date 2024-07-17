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

    using message::actuation::BodySide;
    using message::behaviour::state::Stability;
    using message::input::ButtonLeftDown;
    using message::input::ButtonLeftUp;
    using message::input::ButtonMiddleDown;
    using message::input::ButtonMiddleUp;
    using message::localisation::ResetFieldLocalisation;

    using utility::actuation::tinyrobotics::forward_kinematics_to_servo_map;
    using utility::actuation::tinyrobotics::sensors_to_configuration;
    using utility::input::FrameID;
    using utility::input::ServoID;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::nusight::graph;
    using utility::platform::get_raw_servo;
    using utility::platform::make_packet_error_string;
    using utility::platform::make_servo_hardware_error_string;
    using utility::support::Expression;

    using tinyrobotics::forward_kinematics;

    SensorFilter::SensorFilter(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {


        on<Configuration>("SensorFilter.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            // Button config
            cfg.button_debounce_threshold = config["buttons"]["debounce_threshold"].as<int>();

            // Foot down config
            cfg.foot_down.threshold = config["foot_down"]["threshold"].as<double>();
            cfg.foot_down.method    = config["foot_down"]["method"].as<std::string>();

            // Import URDF model
            nugus_model = tinyrobotics::import_urdf<double, n_servos>(config["urdf_path"].as<std::string>());

            // Configure the Mahony filter
            cfg.initial_Rwt  = rpy_intrinsic_to_mat(Eigen::Vector3d(config["mahony"]["initial_rpy"].as<Expression>()));
            cfg.initial_bias = Eigen::Vector3d(config["mahony"]["initial_bias"].as<Expression>());
            mahony_filter    = MahonyFilter<double>(config["mahony"]["Kp"].as<Expression>(),
                                                 config["mahony"]["Ki"].as<Expression>(),
                                                 cfg.initial_bias,
                                                 cfg.initial_Rwt);

            // Velocity filter config
            cfg.x_cut_off_frequency = config["velocity_low_pass"]["x_cut_off_frequency"].as<double>();
            cfg.y_cut_off_frequency = config["velocity_low_pass"]["y_cut_off_frequency"].as<double>();

            // Initialise the anchor frame (left foot base)
            Hwp.translation().y() = forward_kinematics<double, n_servos>(nugus_model,
                                                                         nugus_model.home_configuration(),
                                                                         std::string("left_foot_base"))
                                        .translation()
                                        .y();

            cfg.use_ground_truth = config["use_ground_truth"].as<bool>();
            cfg.max_servo_change = config["max_servo_change"].as<double>();
        });

        on<Startup>().then([this] {
            // Emit an initial walk state to ensure odometry starts if no other walk state is emitted
            emit(std::make_unique<WalkState>(message::behaviour::state::WalkState::State::UNKNOWN,
                                             Eigen::Vector3d::Zero(),
                                             message::behaviour::state::WalkState::Phase::DOUBLE));
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
        });

        on<Trigger<RawSensors>, Optional<With<Sensors>>, With<Stability>, Single, Priority::HIGH>().then(
            "Main Sensors Loop",
            [this](const RawSensors& raw_sensors,
                   const std::shared_ptr<const Sensors>& previous_sensors,
                   const Stability& stability) {
                auto sensors = std::make_unique<Sensors>();

                // Raw sensors (Accelerometer, Gyroscope, etc.)
                update_raw_sensors(sensors, previous_sensors, raw_sensors);

                // Kinematics (Htw, foot down, CoM, etc.)
                update_kinematics(sensors, raw_sensors);

                // Odometry (Htw and Hrw)
                update_odometry(sensors, previous_sensors, raw_sensors, stability);

                // Graph debug information
                if (log_level <= NUClear::DEBUG) {
                    debug_sensor_filter(sensors, raw_sensors);
                }

                emit(std::move(sensors));
            });

        on<Last<20, Trigger<RawSensors>>>().then(
            [this](const std::list<std::shared_ptr<const RawSensors>>& raw_sensors) {
                // Detect wether a button has been pressed or not in the last 20 messages
                detect_button_press(raw_sensors);
            });

        on<Trigger<ResetFieldLocalisation>>().then([this] {
            // Reset anchor frame
            Hwp                   = Eigen::Isometry3d::Identity();
            Hwp.translation().y() = tinyrobotics::forward_kinematics<double, n_servos>(nugus_model,
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
        if (cfg.foot_down.method == "Z_HEIGHT") {
            const Eigen::Isometry3d Htr(sensors->Htx[FrameID::R_FOOT_BASE]);
            const Eigen::Isometry3d Htl(sensors->Htx[FrameID::L_FOOT_BASE]);
            const Eigen::Isometry3d Hlr = Htl.inverse() * Htr;

            const double rRLl_z = Hlr.translation().z();

            emit(graph("rRLl_z", rRLl_z));

            sensors->feet[BodySide::RIGHT].down = true;
            sensors->feet[BodySide::LEFT].down  = true;
            // The right foot is not down if the z height if the right foot is above a threshold (in left foot space)
            if (rRLl_z > cfg.foot_down.threshold) {
                sensors->feet[BodySide::RIGHT].down = false;
            }
            // The left foot is not down if the z height if the right foot is below a threshold (in left foot space)
            if (rRLl_z < -cfg.foot_down.threshold) {
                sensors->feet[BodySide::LEFT].down = false;
            }
        }
        else if (cfg.foot_down.method == "FSR") {
            // Determine if any two diagonally opposite FSRs are in contact with the ground, which is either fsr1
            // and fsr3, or fsr2 and fsr4. A FSR is in contact with the ground if its value is greater than the
            // certainty threshold
            auto is_foot_down = [](const auto& fsr, const double threshold) {
                return (fsr.fsr1 > threshold && fsr.fsr3 > threshold) || (fsr.fsr2 > threshold && fsr.fsr4 > threshold);
            };
            sensors->feet[BodySide::LEFT].down  = is_foot_down(raw_sensors.fsr.left, cfg.foot_down.threshold);
            sensors->feet[BodySide::RIGHT].down = is_foot_down(raw_sensors.fsr.right, cfg.foot_down.threshold);
        }
        else {
            log<NUClear::WARN>("Unknown foot down method");
        }

        // **************** Planted Foot Information ****************
        if (sensors->feet[BodySide::LEFT].down && sensors->feet[BodySide::RIGHT].down) {
            sensors->planted_foot_phase = message::behaviour::state::WalkState::Phase::DOUBLE;
        }
        else if (sensors->feet[BodySide::LEFT].down && !sensors->feet[BodySide::RIGHT].down) {
            sensors->planted_foot_phase = message::behaviour::state::WalkState::Phase::LEFT;
        }
        else {
            sensors->planted_foot_phase = message::behaviour::state::WalkState::Phase::RIGHT;
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
            const auto& raw_servo       = get_raw_servo(id, raw_sensors);
            const auto& hardware_status = raw_servo.hardware_error;

            // Check for an error on the servo and report it
            if (hardware_status != RawSensors::HardwareError::HARDWARE_OK) {
                NUClear::log<NUClear::WARN>(make_servo_hardware_error_string(raw_servo, id));
            }

            // Determine the current position with potential fallback to the last known good position
            double current_position = raw_servo.present_position;
            if (previous_sensors
                && (fabs(current_position - previous_sensors->servo[id].present_position) > cfg.max_servo_change
                    || hardware_status == RawSensors::HardwareError::MOTOR_ENCODER)) {
                current_position = previous_sensors->servo[id].present_position;
                NUClear::log<NUClear::DEBUG>("Suspected encoder error on servo ",
                                             id,
                                             ": Using last known good position.");
            }

            sensors->servo.emplace_back(
                hardware_status,
                id,
                raw_servo.torque_enabled,
                raw_servo.position_p_gain,
                raw_servo.position_i_gain,
                raw_servo.position_d_gain,
                raw_servo.goal_position,
                raw_servo.profile_velocity,
                current_position,
                /* If there is an encoder error, then use the last known good velocity */
                ((hardware_status == RawSensors::HardwareError::MOTOR_ENCODER) && previous_sensors)
                    ? previous_sensors->servo[id].present_velocity
                    : raw_servo.present_velocity,
                raw_servo.present_current,
                raw_servo.voltage,
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
        // Keep track of the number of presses in the last N frames for debouncing
        int left_count   = 0;
        int middle_count = 0;
        // Count the number of downs in all messages we have
        for (const auto& s : sensors) {
            if (s->buttons.left)
                ++left_count;
            if (s->buttons.middle)
                ++middle_count;
        }
        // Compare to the debounce threshold to determine if we have a down event
        bool new_left_down   = left_count > cfg.button_debounce_threshold;
        bool new_middle_down = middle_count > cfg.button_debounce_threshold;
        // Check for a state change, i.e. a press or release event
        bool left_state_change = left_down != new_left_down;
        bool mid_state_change  = middle_down != new_middle_down;
        // And set the state variable for next time.
        left_down   = new_left_down;
        middle_down = new_middle_down;
        // If we have a state change, emit the appropriate event
        if (left_state_change) {
            if (left_down) {
                log<NUClear::INFO>("Left Button Down");
                emit<Scope::DIRECT>(std::make_unique<ButtonLeftDown>());
            }
            else {
                log<NUClear::INFO>("Left Button Up");
                emit<Scope::DIRECT>(std::make_unique<ButtonLeftUp>());
            }
        }
        if (mid_state_change) {
            if (middle_down) {
                log<NUClear::INFO>("Middle Button Down");
                emit<Scope::DIRECT>(std::make_unique<ButtonMiddleDown>());
            }
            else {
                log<NUClear::INFO>("Middle Button Up");
                emit<Scope::DIRECT>(std::make_unique<ButtonMiddleUp>());
            }
        }
    }

    void SensorFilter::update_odometry(std::unique_ptr<Sensors>& sensors,
                                       const std::shared_ptr<const Sensors>& previous_sensors,
                                       const RawSensors& raw_sensors,
                                       const Stability& stability) {
        // Use ground truth instead of calculating odometry, then return
        if (cfg.use_ground_truth) {
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
            return;
        }

        // Compute time since last update
        const double dt = std::max(
            std::chrono::duration_cast<std::chrono::duration<double>>(
                raw_sensors.timestamp - (previous_sensors ? previous_sensors->timestamp : raw_sensors.timestamp))
                .count(),
            0.0);

        // Perform Mahony update (roll, pitch)
        auto Rwt_mahony = mahony_filter.update(sensors->accelerometer, sensors->gyroscope, dt);
        // Convert the rotation matrix from the mahony method into euler angles
        Eigen::Vector3d rpy_mahony = mat_to_rpy_intrinsic(Rwt_mahony);
        // Remove yaw from mahony filter (prevents it breaking after numerous rotations)
        mahony_filter.set_state(rpy_intrinsic_to_mat(Eigen::Vector3d(rpy_mahony.x(), rpy_mahony.y(), 0.0)));

        // If fallen, calculate roll and pitch but keep yaw and position still
        if (stability <= Stability::FALLING) {
            // Get torso to world
            Eigen::Isometry3d Hwt =
                previous_sensors == nullptr ? Eigen::Isometry3d::Identity() : previous_sensors->Htw.inverse();
            // Htw rotation is combination of Mahony pitch and roll and existing yaw
            Hwt.linear() = rpy_intrinsic_to_mat(
                Eigen::Vector3d(rpy_mahony.x(), rpy_mahony.y(), mat_to_rpy_intrinsic(Hwt.rotation()).z()));
            sensors->Htw = Hwt.inverse();

            // Get robot to world
            sensors->Hrw = previous_sensors == nullptr ? Eigen::Isometry3d::Identity() : previous_sensors->Hrw;

            // Set velocity to zero
            sensors->vTw = Eigen::Vector3d::Zero();

            return;
        }

        // If sensors detected a new foot phase, update the anchor frame
        if (planted_anchor_foot != sensors->planted_foot_phase
            && sensors->planted_foot_phase != WalkState::Phase::DOUBLE) {
            // Update anchor frame to the new planted foot
            switch (planted_anchor_foot.value) {
                case WalkState::Phase::RIGHT:
                    Hwp = Hwp * sensors->Htx[FrameID::R_FOOT_BASE].inverse() * sensors->Htx[FrameID::L_FOOT_BASE];
                    break;
                case WalkState::Phase::LEFT:
                    Hwp = Hwp * sensors->Htx[FrameID::L_FOOT_BASE].inverse() * sensors->Htx[FrameID::R_FOOT_BASE];
                    break;
                default: log<NUClear::WARN>("Anchor frame should not be updated in double support phase"); break;
            }
            // Update our current anchor foot indicator to new foot
            planted_anchor_foot = sensors->planted_foot_phase;
            // Set the z translation, roll and pitch of the anchor frame to 0 as assumed to be on field plane
            Hwp.translation().z() = 0;
            Hwp.linear()          = rpy_intrinsic_to_mat(Eigen::Vector3d(0, 0, mat_to_rpy_intrinsic(Hwp.linear()).z()));
        }

        // Compute torso pose using kinematics from anchor frame (current planted foot)
        Eigen::Isometry3d Hpt = planted_anchor_foot.value == WalkState::Phase::RIGHT
                                    ? Eigen::Isometry3d(sensors->Htx[FrameID::R_FOOT_BASE].inverse())
                                    : Eigen::Isometry3d(sensors->Htx[FrameID::L_FOOT_BASE].inverse());

        // Perform Anchor Update (x, y, z, yaw)
        Eigen::Isometry3d Hwt_anchor = Hwp * Hpt;
        Eigen::Vector3d rpy_anchor   = mat_to_rpy_intrinsic(Hwt_anchor.linear());

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
        // Don't update translation if falling/fallen
        Hwr.translation() = Eigen::Vector3d(Hwt_anchor.translation().x(), Hwt_anchor.translation().y(), 0.0);
        sensors->Hrw      = Hwr.inverse();

        // Low pass filter for torso y velocity
        double y_current     = Hwt.translation().y();
        double y_prev        = previous_sensors ? previous_sensors->Htw.inverse().translation().y() : y_current;
        double y_dot_current = (y_current - y_prev) / dt;
        double y_dot =
            (dt / cfg.y_cut_off_frequency) * y_dot_current + (1 - (dt / cfg.y_cut_off_frequency)) * sensors->vTw.y();

        // Low pass filter for torso x velocity
        double x_current     = Hwt.translation().x();
        double x_prev        = previous_sensors ? previous_sensors->Htw.inverse().translation().x() : x_current;
        double x_dot_current = (x_current - x_prev) / dt;
        double x_dot =
            (dt / cfg.x_cut_off_frequency) * x_dot_current + (1 - (dt / cfg.x_cut_off_frequency)) * sensors->vTw.x();

        // Fuse the velocity estimates
        sensors->vTw = Eigen::Vector3d(x_dot, y_dot, 0);
    }

    void SensorFilter::debug_sensor_filter(std::unique_ptr<Sensors>& sensors, const RawSensors& raw_sensors) {
        // Raw accelerometer and gyroscope information
        emit(graph("Gyroscope", sensors->gyroscope.x(), sensors->gyroscope.y(), sensors->gyroscope.z()));
        emit(
            graph("Accelerometer", sensors->accelerometer.x(), sensors->accelerometer.y(), sensors->accelerometer.z()));

        // Foot down sensors state for each foot
        emit(graph(fmt::format("Foot Down/{}/Left", std::string(cfg.foot_down.method)),
                   sensors->feet[BodySide::LEFT].down));
        emit(graph(fmt::format("Foot Down/{}/Right", std::string(cfg.foot_down.method)),
                   sensors->feet[BodySide::RIGHT].down));
        emit(graph("Foot down phase", int(sensors->planted_foot_phase)));
        emit(graph("Anchor foot", int(planted_anchor_foot)));

        // Odometry information
        Eigen::Isometry3d Hwt    = Eigen::Isometry3d(sensors->Htw).inverse();
        Eigen::Vector3d est_rTWw = Hwt.translation();
        Eigen::Vector3d est_Rwt  = mat_to_rpy_intrinsic(Hwt.rotation());
        emit(graph("Hwt est translation (rTWw)", est_rTWw.x(), est_rTWw.y(), est_rTWw.z()));
        emit(graph("Rwt est angles (rpy)", est_Rwt.x(), est_Rwt.y(), est_Rwt.z()));
        emit(graph("vTw est", sensors->vTw.x(), sensors->vTw.y(), sensors->vTw.z()));

        Eigen::Isometry3d Hwr    = Eigen::Isometry3d(sensors->Hrw).inverse();
        Eigen::Vector3d est_rTRw = Hwr.translation();
        Eigen::Vector3d est_Rrw  = mat_to_rpy_intrinsic(Hwr.rotation());
        emit(graph("Hwr est translation (rTRw)", est_rTRw.x(), est_rTRw.y(), est_rTRw.z()));
        emit(graph("Rrw est angles (rpy)", est_Rrw.x(), est_Rrw.y(), est_Rrw.z()));

        Eigen::Isometry3d Hwp    = sensors->Hwp;
        Eigen::Vector3d est_rPWw = Hwp.translation();
        Eigen::Vector3d est_Rpw  = mat_to_rpy_intrinsic(Hwp.rotation());
        emit(graph("Hwp est translation (rTPw)", est_rPWw.x(), est_rPWw.y(), est_rPWw.z()));
        emit(graph("Rpw est angles (rpy)", est_Rpw.x(), est_Rpw.y(), est_Rpw.z()));

        Eigen::Vector3d vTw = sensors->vTw;
        emit(graph("vTw est", vTw.x(), vTw.y(), vTw.z()));

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
