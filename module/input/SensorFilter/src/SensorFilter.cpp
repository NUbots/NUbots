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

#include "extension/Configuration.hpp"

#include "message/actuation/BodySide.hpp"
#include "message/input/Sensors.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/actuation/ForwardKinematics.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/euler.hpp"
#include "utility/math/filter/inekf/InEKF.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"
namespace module::input {

    using extension::Configuration;

    using message::actuation::BodySide;
    using message::actuation::KinematicsModel;
    using message::input::Sensors;
    using message::platform::ButtonLeftDown;
    using message::platform::ButtonLeftUp;
    using message::platform::ButtonMiddleDown;
    using message::platform::ButtonMiddleUp;
    using message::platform::RawSensors;

    using utility::actuation::kinematics::calculateAllPositions;
    using utility::actuation::kinematics::calculateCentreOfMass;
    using utility::actuation::kinematics::calculateInertialTensor;
    using utility::input::ServoID;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::nusight::graph;
    using utility::support::Expression;


    std::string makeErrorString(const std::string& src, uint errorCode) {
        std::stringstream s;

        s << "Error on ";
        s << src;
        s << ":";

        if ((errorCode & RawSensors::Error::INPUT_VOLTAGE) != 0u) {
            s << " Input Voltage ";
        }
        if ((errorCode & RawSensors::Error::ANGLE_LIMIT) != 0u) {
            s << " Angle Limit ";
        }
        if ((errorCode & RawSensors::Error::OVERHEATING) != 0u) {
            s << " Overheating ";
        }
        if ((errorCode & RawSensors::Error::OVERLOAD) != 0u) {
            s << " Overloaded ";
        }
        if ((errorCode & RawSensors::Error::INSTRUCTION) != 0u) {
            s << " Bad Instruction ";
        }
        if ((errorCode & RawSensors::Error::CORRUPT_DATA) != 0u) {
            s << " Corrupt Data ";
        }
        if ((errorCode & RawSensors::Error::TIMEOUT) != 0u) {
            s << " Timeout ";
        }

        return s.str();
    }

    SensorFilter::SensorFilter(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), theta(Eigen::Vector3d::Zero()) {

        on<Configuration>("SensorFilter.yaml").then([this](const Configuration& cfg) {
            log_level = cfg["log_level"].as<NUClear::LogLevel>();

            // Button config
            config.buttons.debounceThreshold = cfg["buttons"]["debounce_threshold"].as<int>();

            // Foot down config
            const FootDownMethod method = cfg["foot_down"]["method"].as<std::string>();
            std::map<FootDownMethod, float> thresholds;
            for (const auto& threshold : cfg["foot_down"]["known_methods"]) {
                thresholds[threshold["name"].as<std::string>()] = threshold["certainty_threshold"].as<float>();
            }
            config.footDown.set_method(method, thresholds);

            // SET INITIAL PARAMETERS FOR THE INEKF

            config.inekf.initial_orientation = cfg["inekf"]["initial"]["orientation"].as<Expression>();
            config.inekf.initial_velocity    = cfg["inekf"]["initial"]["velocity"].as<Expression>();
            config.inekf.initial_position    = cfg["inekf"]["initial"]["position"].as<Expression>();
            config.inekf.initial_gyro_bias   = cfg["inekf"]["initial"]["gyro_bias"].as<Expression>();
            config.inekf.initial_acc_bias    = cfg["inekf"]["initial"]["acc_bias"].as<Expression>();

            utility::math::filter::inekf::RobotState initial_state{};
            initial_state.set_rotation(config.inekf.initial_orientation);
            initial_state.set_velocity(config.inekf.initial_velocity);
            initial_state.set_position(config.inekf.initial_position);
            initial_state.set_gyroscope_bias(config.inekf.initial_gyro_bias);
            initial_state.set_accelerometer_bias(config.inekf.initial_acc_bias);

            filter.set_state(initial_state);

            // SET NOISE PARAMETERS FOR THE INEKF
            config.inekf.noise_gyro         = cfg["inekf"]["noise"]["gyro"].as<double>();
            config.inekf.noise_acc          = cfg["inekf"]["noise"]["acc"].as<double>();
            config.inekf.noise_gyro_bias    = cfg["inekf"]["noise"]["gyro_bias"].as<double>();
            config.inekf.noise_acc_bias     = cfg["inekf"]["noise"]["acc_bias"].as<double>();
            config.inekf.noise_foot_sensors = cfg["inekf"]["noise"]["foot_sensors"].as<double>();

            utility::math::filter::inekf::NoiseParams noise_params(config.inekf.noise_gyro,
                                                                   config.inekf.noise_acc,
                                                                   config.inekf.noise_gyro_bias,
                                                                   config.inekf.noise_acc_bias,
                                                                   config.inekf.noise_foot_sensors);

            filter.set_noise_params(noise_params);

            // Set our initial position
            rTWw = cfg["inekf"]["initial"]["position"].as<Expression>();

            // Don't filter any sensors until we have initialised the filter
            update_loop.disable();
            reset_filter.store(true);
        });

        on<Configuration>("FootDownNetwork.yaml").then([this](const Configuration& config) {
            // Foot load sensor config
            load_sensor = VirtualLoadSensor<float>(config);
        });

        on<Last<20, Trigger<RawSensors>>, With<KinematicsModel>, Single>().then(
            [this](const std::list<std::shared_ptr<const RawSensors>>& sensors, const KinematicsModel& model) {
                // If we need to reset the filter, do that here
                if (reset_filter.load()) {
                    Eigen::Vector3d acc  = Eigen::Vector3d::Zero();
                    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
                    Eigen::Vector3d rMFt = Eigen::Vector3d::Zero();

                    for (const auto& s : sensors) {
                        Sensors filtered_sensors{};

                        // Accumulate accelerometer and gyroscope readings
                        acc += s->accelerometer.cast<double>();
                        gyro += s->gyroscope.cast<double>();
                        // Make sure we have servo positions
                        for (uint32_t id = 0; id < 20; ++id) {
                            const auto& original = utility::platform::getRawServo(id, *s);
                            // Add the sensor values to the system properly
                            filtered_sensors.servo.emplace_back(0,
                                                                id,
                                                                original.torque_enabled,
                                                                original.p_gain,
                                                                original.i_gain,
                                                                original.d_gain,
                                                                original.goal_position,
                                                                original.moving_speed,
                                                                original.present_position,
                                                                original.present_speed,
                                                                original.load,
                                                                original.voltage,
                                                                static_cast<float>(original.temperature));
                        }

                        // Calculate forward kinematics
                        const auto Htx = calculateAllPositions(model, filtered_sensors);
                        for (const auto& entry : Htx) {
                            filtered_sensors.Htx[entry.first] = entry.second.matrix();
                        }

                        // Calculate the average length of both legs from the torso and accumulate this measurement
                        const Eigen::Isometry3d Htr(filtered_sensors.Htx[ServoID::R_ANKLE_ROLL]);
                        const Eigen::Isometry3d Htl(filtered_sensors.Htx[ServoID::L_ANKLE_ROLL]);
                        const Eigen::Vector3d rTFt = (Htr.translation() + Htl.translation()) * 0.5;

                        // Accumulator CoM readings
                        rMFt += calculateCentreOfMass(model, filtered_sensors.Htx).head<3>() + rTFt;
                    }

                    // Average all accumulated readings
                    acc /= static_cast<double>(sensors.size());
                    gyro /= static_cast<double>(sensors.size());
                    rMFt /= static_cast<double>(sensors.size());

                    // Average time per sensor reading
                    double deltaT = std::chrono::duration_cast<std::chrono::duration<double>>(
                                        sensors.back()->timestamp - sensors.front()->timestamp)
                                        .count()
                                    / static_cast<double>(sensors.size());

                    // Find the rotation from the average accelerometer reading to world UnitZ
                    // Rotating from torso acceleration vector to world z vector ===> this makes it Rwt and not Rtw
                    Eigen::Quaterniond Rwt = Eigen::Quaterniond::FromTwoVectors(acc, Eigen::Vector3d::UnitZ());
                    Rwt.normalize();
                }

                int leftCount   = 0;
                int middleCount = 0;

                // If we have any downs in the last 20 frames then we are button pushed
                for (const auto& s : sensors) {
                    if (s->buttons.left && (s->platform_error_flags == 0u)) {
                        ++leftCount;
                    }
                    if (s->buttons.middle && (s->platform_error_flags == 0u)) {
                        ++middleCount;
                    }
                }

                bool newLeftDown   = leftCount > config.buttons.debounceThreshold;
                bool newMiddleDown = middleCount > config.buttons.debounceThreshold;

                if (newLeftDown != leftDown) {

                    leftDown = newLeftDown;

                    if (newLeftDown) {
                        log("Left Button Down");
                        emit(std::make_unique<ButtonLeftDown>());
                    }
                    else {
                        log("Left Button Up");
                        emit(std::make_unique<ButtonLeftUp>());
                    }
                }
                if (newMiddleDown != middleDown) {

                    middleDown = newMiddleDown;

                    if (newMiddleDown) {
                        log("Middle Button Down");
                        emit(std::make_unique<ButtonMiddleDown>());
                    }
                    else {
                        log("Middle Button Up");
                        emit(std::make_unique<ButtonMiddleUp>());
                    }
                }
            });

        update_loop =
            on<Trigger<RawSensors>, Optional<With<Sensors>>, With<KinematicsModel>, Single, Priority::HIGH>()
                .then(
                    "Main Sensors Loop",
                    [this](const RawSensors& input,
                           const std::shared_ptr<const Sensors>& previousSensors,
                           const KinematicsModel& kinematicsModel) {
                        auto sensors = std::make_unique<Sensors>();

                        /************************************************
                         *                 Raw Sensors                  *
                         ************************************************/

                        // Set our timestamp to when the data was read
                        sensors->timestamp = input.timestamp;

                        sensors->voltage = input.voltage;


                        // This checks for an error on the platform and reports it
                        if (input.platform_error_flags != RawSensors::Error::OK) {
                            NUClear::log<NUClear::WARN>(makeErrorString("Platform", input.platform_error_flags));
                        }

                        // Output errors on the FSRs
                        if (input.fsr.left.error_flags != RawSensors::Error::OK) {
                            NUClear::log<NUClear::WARN>(makeErrorString("Left FSR", input.fsr.left.error_flags));
                        }

                        if (input.fsr.right.error_flags != RawSensors::Error::OK) {
                            NUClear::log<NUClear::WARN>(makeErrorString("Right FSR", input.fsr.right.error_flags));
                        }

                        // Read through all of our sensors
                        for (uint32_t id = 0; id < 20; ++id) {
                            const auto& original = utility::platform::getRawServo(id, input);
                            const auto& error    = original.error_flags;

                            // Check for an error on the servo and report it
                            if (error != RawSensors::Error::OK) {
                                std::stringstream s;
                                s << "Error on Servo " << (id + 1) << " (" << static_cast<ServoID>(id) << "):";

                                if ((error & RawSensors::Error::INPUT_VOLTAGE) != 0u) {
                                    s << " Input Voltage - " << original.voltage;
                                }
                                if ((error & RawSensors::Error::ANGLE_LIMIT) != 0u) {
                                    s << " Angle Limit - " << original.present_position;
                                }
                                if ((error & RawSensors::Error::OVERHEATING) != 0u) {
                                    s << " Overheating - " << original.temperature;
                                }
                                if ((error & RawSensors::Error::OVERLOAD) != 0u) {
                                    s << " Overloaded - " << original.load;
                                }
                                if ((error & RawSensors::Error::INSTRUCTION) != 0u) {
                                    s << " Bad Instruction ";
                                }
                                if ((error & RawSensors::Error::CORRUPT_DATA) != 0u) {
                                    s << " Corrupt Data ";
                                }
                                if ((error & RawSensors::Error::TIMEOUT) != 0u) {
                                    s << " Timeout ";
                                }

                                NUClear::log<NUClear::WARN>(s.str());
                            }
                            // If current Sensors message for this servo has an error and we have a previous sensors
                            // message available, then we use our previous sensor values with some updates
                            if (error != RawSensors::Error::OK && previousSensors) {
                                // Add the sensor values to the system properly
                                sensors->servo.emplace_back(error,
                                                            id,
                                                            original.torque_enabled,
                                                            original.p_gain,
                                                            original.i_gain,
                                                            original.d_gain,
                                                            original.goal_position,
                                                            original.moving_speed,
                                                            previousSensors->servo[id].present_position,
                                                            previousSensors->servo[id].present_velocity,
                                                            previousSensors->servo[id].load,
                                                            previousSensors->servo[id].voltage,
                                                            previousSensors->servo[id].temperature);
                            }
                            // Otherwise we can just use the new values as is
                            else {
                                // Add the sensor values to the system properly
                                sensors->servo.emplace_back(error,
                                                            id,
                                                            original.torque_enabled,
                                                            original.p_gain,
                                                            original.i_gain,
                                                            original.d_gain,
                                                            original.goal_position,
                                                            original.moving_speed,
                                                            original.present_position,
                                                            original.present_speed,
                                                            original.load,
                                                            original.voltage,
                                                            static_cast<float>(original.temperature));
                            }
                        }

                        /************************************************
                         *          Accelerometer and Gyroscope         *
                         ************************************************/

                        // We assume that the accelerometer and gyroscope are oriented to conform with the standard
                        // coordinate system x-axis out the front of the robot y-axis to the left z-axis up
                        //
                        // For the accelerometer the orientation should be as follows
                        // x axis reports a +1g acceleration when robot is laying on its back
                        // y axis reports a +1g acceleration when robot is laying on its right side
                        // z axis reports a +1g acceleration when robot is vertical

                        // If we have a previous sensors and our platform has errors then reuse our last sensor value
                        if ((input.platform_error_flags != 0u) && previousSensors) {
                            sensors->accelerometer = previousSensors->accelerometer;
                        }
                        else {
                            sensors->accelerometer = input.accelerometer.cast<double>();
                        }

                        // If we have a previous Sensors message and (our platform has errors or we are spinning too
                        // quickly), then reuse our last sensor value
                        if (previousSensors
                            && ((input.platform_error_flags != 0u)
                                // One of the gyros would occasionally throw massive numbers without an error flag
                                // If our hardware is working as intended, it should never read that we're spinning at 2
                                // revs/s
                                || input.gyroscope.norm() > 4.0 * M_PI)) {
                            NUClear::log<NUClear::WARN>("Bad gyroscope value", input.gyroscope.norm());
                            sensors->gyroscope = previousSensors->gyroscope;
                        }
                        else {
                            sensors->gyroscope = input.gyroscope.cast<double>();
                        }

                        // Add gyro and acc graphs if in debug
                        if (log_level <= NUClear::DEBUG) {
                            emit(graph("Gyroscope",
                                       sensors->gyroscope.x(),
                                       sensors->gyroscope.y(),
                                       sensors->gyroscope.z()));
                            emit(graph("Accelerometer",
                                       sensors->accelerometer.x(),
                                       sensors->accelerometer.y(),
                                       sensors->accelerometer.z()));
                        }

                        /************************************************
                         *               Buttons and LEDs               *
                         ************************************************/
                        sensors->button.reserve(2);
                        sensors->button.emplace_back(0, input.buttons.left);
                        sensors->button.emplace_back(1, input.buttons.middle);
                        sensors->led.reserve(5);
                        sensors->led.emplace_back(0, input.led_panel.led2 ? 0xFF0000 : 0);
                        sensors->led.emplace_back(1, input.led_panel.led3 ? 0xFF0000 : 0);
                        sensors->led.emplace_back(2, input.led_panel.led4 ? 0xFF0000 : 0);
                        sensors->led.emplace_back(3, input.head_led.RGB);  // Head
                        sensors->led.emplace_back(4, input.eye_led.RGB);   // Eye

                        /************************************************
                         *                  Kinematics                  *
                         ************************************************/

                        // Htx is a map from ServoID to homogeneous transforms from each ServoID to the torso
                        auto Htx = calculateAllPositions(kinematicsModel, *sensors);
                        for (const auto& entry : Htx) {
                            sensors->Htx[entry.first] = entry.second.matrix();
                        }

                        /************************************************
                         *                  Mass Model                  *
                         ************************************************/
                        sensors->rMTt           = calculateCentreOfMass(kinematicsModel, sensors->Htx);
                        sensors->inertia_tensor = calculateInertialTensor(kinematicsModel, sensors->Htx);

                        /************************************************
                         *            Foot down information             *
                         ************************************************/
                        sensors->feet.resize(2);
                        sensors->feet[BodySide::RIGHT].down = true;
                        sensors->feet[BodySide::LEFT].down  = true;

                        std::array<bool, 2> feet_down = {true, true};

                        // Calculate values needed for Z_HEIGHT method
                        const Eigen::Isometry3d Htr(sensors->Htx[ServoID::R_ANKLE_ROLL]);
                        const Eigen::Isometry3d Htl(sensors->Htx[ServoID::L_ANKLE_ROLL]);
                        const Eigen::Isometry3d Hlr = Htl.inverse() * Htr;
                        const Eigen::Vector3d rRLl  = Hlr.translation();

                        switch (config.footDown.method()) {
                            case FootDownMethod::LOAD:
                                // Use our virtual load sensor class to work out which feet are down
                                feet_down = load_sensor.updateFeet(*sensors);
                                break;
                            case FootDownMethod::Z_HEIGHT:
                                // Right foot is below left foot in left foot space
                                if (rRLl.z() < -config.footDown.threshold()) {
                                    feet_down[BodySide::RIGHT] = true;
                                    feet_down[BodySide::LEFT]  = false;
                                }
                                // Right foot is above left foot in left foot space
                                else if (rRLl.z() > config.footDown.threshold()) {
                                    feet_down[BodySide::RIGHT] = false;
                                    feet_down[BodySide::LEFT]  = true;
                                }
                                // Right foot and left foot are roughly the same height in left foot space
                                else {
                                    feet_down[BodySide::RIGHT] = true;
                                    feet_down[BodySide::LEFT]  = true;
                                }
                                break;
                            case FootDownMethod::FSR:
                                // For a foot to be on the ground we want a minimum of 2 diagonally opposite studs
                                // in contact with the ground
                                // So fsr1 and fsr3, or fsr2 and fsr4
                                //
                                // A FSR is in contact with the ground if its value is greater than the certainty
                                // threshold

                                feet_down[BodySide::LEFT] =
                                    (((input.fsr.left.fsr1 > config.footDown.threshold())
                                      && (input.fsr.left.fsr3 > config.footDown.threshold()))
                                     || ((input.fsr.left.fsr2 > config.footDown.threshold())
                                         && (input.fsr.left.fsr4 > config.footDown.threshold())));

                                feet_down[BodySide::RIGHT] =
                                    (((input.fsr.right.fsr1 > config.footDown.threshold())
                                      && (input.fsr.right.fsr3 > config.footDown.threshold()))
                                     || ((input.fsr.right.fsr2 > config.footDown.threshold())
                                         && (input.fsr.right.fsr4 > config.footDown.threshold())));
                                break;
                            default: log<NUClear::WARN>("Unknown foot down method"); break;
                        }

                        if (log_level <= NUClear::DEBUG) {
                            emit(graph(fmt::format("Sensor/Foot Down/{}/Left", std::string(config.footDown.method())),
                                       feet_down[BodySide::LEFT]));
                            emit(graph(fmt::format("Sensor/Foot Down/{}/Right", std::string(config.footDown.method())),
                                       feet_down[BodySide::RIGHT]));
                        }

                        sensors->feet[BodySide::RIGHT].down = feet_down[BodySide::RIGHT];
                        sensors->feet[BodySide::LEFT].down  = feet_down[BodySide::LEFT];

                        /************************************************
                         *             Motion (IMU+Odometry)            *
                         ************************************************/

                        // Time for filter
                        using namespace std::chrono;
                        const double deltaT = std::max(
                            duration_cast<duration<double>>(
                                input.timestamp - (previousSensors ? previousSensors->timestamp : input.timestamp))
                                .count(),
                            0.0);

                        // IMU data for filter
                        Eigen::Vector3d gyro = previousSensors ? previousSensors->gyroscope : Eigen::Vector3d::Zero();
                        Eigen::Vector3d acc =
                            previousSensors ? previousSensors->accelerometer : Eigen::Vector3d::Zero();

                        filter.propagate(gyro, acc, deltaT);

                        // Contact data for filter
                        filter.set_contacts({{0, feet_down[BodySide::RIGHT]}, {1, feet_down[BodySide::LEFT]}});

                        // Kinematics data for filter
                        utility::math::filter::inekf::kinematics measured_kinematics;
                        measured_kinematics.emplace_back(
                            utility::math::filter::inekf::KinematicPose{0,
                                                                        Htr.matrix(),
                                                                        Eigen::Matrix<double, 6, 6>::Zero()});
                        measured_kinematics.emplace_back(
                            utility::math::filter::inekf::KinematicPose{1,
                                                                        Htl.matrix(),
                                                                        Eigen::Matrix<double, 6, 6>::Zero()});
                        filter.correct_kinematics(measured_kinematics);


                        // Filter is not reliable, just use previous sensors
                        if (reset_filter.load()) {
                            if (previousSensors) {
                                sensors->Htw = previousSensors->Htw;
                                sensors->Hgt = previousSensors->Hgt;
                            }
                        }
                        else {
                            /************************************************
                             *       Torso CoM Position in World (rMWw)     *
                             ************************************************/
                            bool update_done = false;

                            for (const auto& side : {BodySide::LEFT, BodySide::RIGHT}) {
                                const bool& foot_down      = sensors->feet[side].down;
                                const bool& prev_foot_down = previous_foot_down[side];

                                // Get the Foot to Torso transform for this foot
                                const Eigen::Isometry3d Htf(
                                    sensors
                                        ->Htx[side == BodySide::LEFT ? ServoID::L_ANKLE_ROLL : ServoID::R_ANKLE_ROLL]);

                                // Calculate our current Foot to CoM vector for this foot
                                const Eigen::Vector3d current_rMFt = Htf.translation() + sensors->rMTt.head<3>();

                                // We just put this foot on the ground (i.e. it wasn't on the ground in the last time
                                // step)
                                if (foot_down && !prev_foot_down) {
                                    // Update our Foot to CoM vector for this foot
                                    rMFt[side]               = current_rMFt;
                                    previous_foot_down[side] = true;
                                }
                                // Our foot is on the ground and was also on the ground in the last time step
                                else if (foot_down && prev_foot_down) {
                                    // If both feet are on the ground then we don't need to do another update
                                    if (!update_done) {
                                        // The difference between our current and previous Foot to torso CoM vectors is
                                        // how much our torso has moved in the last time step in torso space We need to
                                        // rotate this into world space to update our current Torso CoM position
                                        const Eigen::Quaterniond& Rwt =
                                            Eigen::Quaterniond(filter.get_state().get_rotation());
                                        const Eigen::Vector3d rMFt_update = current_rMFt - rMFt[side];
                                        const Eigen::Quaterniond q(
                                            Eigen::Vector4d(rMFt_update.x(), rMFt_update.y(), rMFt_update.z(), 0.0));
                                        rTWw += (Rwt * q * Rwt.conjugate()).vec();

                                        // Make sure we don't do another update
                                        update_done = true;
                                    }
                                    previous_foot_down[side] = true;
                                    rMFt[side]               = current_rMFt;
                                }
                                else {
                                    previous_foot_down[side] = false;
                                }
                            }

                            /************************************************
                             *       Construct Odometry Output (Htw)        *
                             ************************************************/
                            // Map from world to torso coordinates (Rtw)
                            Eigen::Isometry3d Hwt;
                            Hwt.linear()      = filter.get_state().get_rotation();
                            Hwt.translation() = filter.get_state().get_position();
                            sensors->Htw      = Hwt.inverse().matrix();

                            // If there is ground truth data, determine the error in the odometry calculation
                            // and emit graphs of those errors
                            if (input.odometry_ground_truth.exists) {
                                Eigen::Isometry3d true_Htw(input.odometry_ground_truth.Htw);

                                // Determine translational distance error
                                Eigen::Vector3d est_rWTt   = Hwt.inverse().translation();
                                Eigen::Vector3d true_rWTt  = true_Htw.translation();
                                Eigen::Vector3d error_rWTt = (true_rWTt - est_rWTt).cwiseAbs();

                                // Determine yaw, pitch and roll error
                                Eigen::Vector3d true_Rtw  = MatrixToEulerIntrinsic(true_Htw.rotation());
                                Eigen::Vector3d est_Rtw   = MatrixToEulerIntrinsic(Hwt.inverse().rotation());
                                Eigen::Vector3d error_Rtw = (true_Rtw - est_Rtw).cwiseAbs();

                                double quat_rot_error = Eigen::Quaterniond(true_Htw.linear() * Hwt.linear()).w();

                                // Graph translation and its error
                                emit(graph("Htw est translation (rWTt)", est_rWTt.x(), est_rWTt.y(), est_rWTt.z()));
                                emit(graph("Htw true translation (rWTt)", true_rWTt.x(), true_rWTt.y(), true_rWTt.z()));
                                emit(graph("Htw translation error", error_rWTt.x(), error_rWTt.y(), error_rWTt.z()));

                                // Graph angles and error
                                emit(graph("Rtw est angles (rpy)", est_Rtw.x(), est_Rtw.y(), est_Rtw.z()));
                                emit(graph("Rtw true angles (rpy)", true_Rtw.x(), true_Rtw.y(), true_Rtw.z()));
                                emit(graph("Rtw error (rpy)", error_Rtw.x(), error_Rtw.y(), error_Rtw.z()));
                                emit(graph("Quaternion rotational error", quat_rot_error));
                            }

                            /************************************************
                             *                  Kinematics Horizon          *
                             ************************************************/
                            Eigen::Isometry3d Rwt(sensors->Htw.inverse());
                            // remove translation components from the transform
                            Rwt.translation() = Eigen::Vector3d::Zero();
                            Eigen::Isometry3d Rgt(
                                Eigen::AngleAxisd(-Rwt.rotation().eulerAngles(0, 1, 2).z(), Eigen::Vector3d::UnitZ())
                                * Rwt);
                            // sensors->Hgt : Mat size [4x4] (default identity)
                            // createRotationZ : Mat size [3x3]
                            // Rwt : Mat size [3x3]
                            sensors->Hgt = Rgt.matrix();
                        }

                        if (log_level <= NUClear::DEBUG) {
                            const Eigen::Isometry3d Htl(sensors->Htx[ServoID::L_ANKLE_ROLL]);
                            const Eigen::Isometry3d Htr(sensors->Htx[ServoID::R_ANKLE_ROLL]);
                            Eigen::Matrix<double, 3, 3> Rtl     = Htl.linear();
                            Eigen::Matrix<double, 3, 1> Rtl_rpy = MatrixToEulerIntrinsic(Rtl);
                            emit(graph("Left Foot Actual Position", Htl(0, 3), Htl(1, 3), Htl(2, 3)));
                            emit(graph("Left Foot Actual Orientation (r,p,y)", Rtl_rpy.x(), Rtl_rpy.y(), Rtl_rpy.z()));
                            Eigen::Matrix<double, 3, 3> Rtr     = Htr.linear();
                            Eigen::Matrix<double, 3, 1> Rtr_rpy = MatrixToEulerIntrinsic(Rtr);
                            emit(graph("Right Foot Actual Position", Htr(0, 3), Htr(1, 3), Htr(2, 3)));
                            emit(graph("Right Foot Actual Orientation (r,p,y)", Rtr_rpy.x(), Rtr_rpy.y(), Rtr_rpy.z()));
                        }

                        emit(std::move(sensors));
                    })
                .disable();
    }
}  // namespace module::input
