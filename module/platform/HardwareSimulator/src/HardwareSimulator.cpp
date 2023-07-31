
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

#include "HardwareSimulator.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <mutex>

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"
#include "message/input/Sensors.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/input/LinkID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/angle.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::platform {

    using extension::Configuration;

    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;
    using message::input::Sensors;
    using message::platform::RawSensors;

    using utility::input::LinkID;
    using utility::input::ServoID;
    using utility::support::Expression;

    HardwareSimulator::HardwareSimulator(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        /*
         CM740 Data
         */
        // Read our Error code
        sensors.platform_error_flags = 0;

        // LED Panel
        sensors.led_panel.led2 = false;
        sensors.led_panel.led3 = false;
        sensors.led_panel.led4 = false;

        // Head LED
        sensors.head_led.RGB = 0;

        // Head LED
        sensors.eye_led.RGB = 0;

        // Buttons
        sensors.buttons.left   = false;
        sensors.buttons.middle = false;

        // Gyroscope (in radians/second)
        sensors.gyroscope = Eigen::Vector3f::Zero();

        /*
         Force Sensitive Resistor Data
         */

        // Right Sensor
        // Error
        sensors.fsr.right.error_flags = 0;

        // Sensors
        sensors.fsr.right.fsr1 = 1;
        sensors.fsr.right.fsr2 = 1;
        sensors.fsr.right.fsr3 = 1;
        sensors.fsr.right.fsr4 = 1;

        // Centre
        sensors.fsr.right.centre_x = 0;
        sensors.fsr.right.centre_y = 0;

        // Left Sensor
        // Error
        sensors.fsr.left.error_flags = 0;

        // Sensors
        sensors.fsr.left.fsr1 = 1;
        sensors.fsr.left.fsr2 = 1;
        sensors.fsr.left.fsr3 = 1;
        sensors.fsr.left.fsr4 = 1;

        // Centre
        sensors.fsr.left.centre_x = 0;
        sensors.fsr.left.centre_y = 0;

        /*
         Servos
         */

        for (int i = 0; i < 20; ++i) {
            // Get a reference to the servo we are populating
            RawSensors::Servo& servo = utility::platform::getRawServo(i, sensors);

            // Error code
            servo.error_flags = 0;

            // Booleans
            servo.torque_enabled = true;

            // Gain
            servo.position_d_gain = 0;
            servo.position_i_gain = 0;
            servo.position_p_gain = 0;
            servo.velocity_i_gain = 0;
            servo.velocity_p_gain = 0;

            // Targets
            servo.goal_position    = 0;
            servo.profile_velocity = M_PI_4;

            // Present Data
            servo.present_position = 0;
            servo.present_velocity = 0;
            servo.present_current  = 0;

            // Diagnostic Information
            servo.voltage     = 0;
            servo.temperature = 0;
        }

        on<Configuration>("HardwareSimulator.yaml")
            .then("Hardware Simulator Config", [this](const Configuration& config) {
                imu_drift_rate      = config["imu_drift_rate"].as<float>();
                noise.accelerometer = config["noise"]["accelerometer"].as<Expression>();
                noise.gyroscope     = config["noise"]["gyroscope"].as<Expression>();
                bodyTilt            = config["body_tilt"].as<Expression>();
            });

        on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Optional<With<Sensors>>, Single>().then(
            [this](const std::shared_ptr<const Sensors>& previousSensors) {
                if (previousSensors) {
                    Eigen::Isometry3d Hf_rt(previousSensors->Htx[LinkID::R_ANKLE_ROLL]);
                    Eigen::Isometry3d Hf_lt(previousSensors->Htx[LinkID::L_ANKLE_ROLL]);
                    Eigen::Vector3d torsoFromRightFoot = -Hf_rt.rotation().transpose() * Hf_rt.translation();
                    Eigen::Vector3d torsoFromLeftFoot  = -Hf_lt.rotation().transpose() * Hf_lt.translation();

                    if (torsoFromRightFoot.z() > torsoFromLeftFoot.z()) {
                        setLeftFootDown(false);
                        setRightFootDown(true);
                    }
                    else if (torsoFromRightFoot.z() < torsoFromLeftFoot.z()) {
                        setLeftFootDown(true);
                        setRightFootDown(false);
                    }
                    else {
                        setLeftFootDown(true);
                        setRightFootDown(true);
                    }
                }

                for (int i = 0; i < 20; ++i) {
                    auto& servo       = utility::platform::getRawServo(i, sensors);
                    float movingSpeed = servo.profile_velocity == 0 ? 0.1 : servo.profile_velocity / UPDATE_FREQUENCY;
                    movingSpeed       = movingSpeed > 0.1 ? 0.1 : movingSpeed;


                    if (std::abs(servo.present_position - servo.goal_position) < movingSpeed) {
                        servo.present_position = servo.goal_position;
                    }
                    else {
                        Eigen::Vector3d present(std::cos(servo.present_position),
                                                std::sin(servo.present_position),
                                                0.0);
                        Eigen::Vector3d goal(std::cos(servo.goal_position), std::sin(servo.goal_position), 0.0);

                        Eigen::Vector3d cross = present.cross(goal);
                        if (cross.z() > 0) {
                            servo.present_position =
                                utility::math::angle::normalizeAngle(servo.present_position + movingSpeed);
                        }
                        else {
                            servo.present_position =
                                utility::math::angle::normalizeAngle(servo.present_position - movingSpeed);
                        }
                    }
                }

                sensors.gyroscope     = Eigen::Vector3f(0.0f, 0.0f, imu_drift_rate);
                sensors.accelerometer = Eigen::Vector3f(-9.8 * std::sin(bodyTilt), 0.0, 9.8 * std::cos(bodyTilt));
                sensors.timestamp     = NUClear::clock::now();

                // Add some noise so that sensor fusion doesnt converge to a singularity
                auto sensors_message = std::make_unique<RawSensors>(sensors);
                addNoise(sensors_message);

                // Send our nicely computed sensor data out to the world
                emit(std::move(sensors_message));
            });

        // This trigger writes the servo positions to the hardware
        on<Trigger<ServoTargets>>().then([this](const ServoTargets& commands) {
            for (const auto& command : commands.targets) {

                // Calculate our moving speed
                float diff = utility::math::angle::difference(
                    command.position,
                    utility::platform::getRawServo(command.id, sensors).present_position);
                NUClear::clock::duration duration = command.time - NUClear::clock::now();

                float speed = 0.0f;
                if (duration.count() > 0) {
                    speed = diff / (double(duration.count()) / double(NUClear::clock::period::den));
                }

                // Set our variables
                auto& servo            = utility::platform::getRawServo(command.id, sensors);
                servo.profile_velocity = speed;
                servo.goal_position    = utility::math::angle::normalizeAngle(command.position);
            }
        });

        on<Trigger<ServoTarget>>().then([this](const ServoTarget& command) {
            auto commandList = std::make_unique<ServoTargets>();
            commandList->targets.push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::DIRECT>(commandList);
        });
    }

    float centered_noise() {
        return rand() / float(RAND_MAX) - 0.5f;
    }

    void HardwareSimulator::addNoise(std::unique_ptr<RawSensors>& sensors) const {
        // TODO(HardwareTeam,DevOpsTeam): Use a more standard c++ random generator.
        sensors->accelerometer += noise.accelerometer * centered_noise();
        sensors->gyroscope += noise.gyroscope * centered_noise();
    }

    void HardwareSimulator::setRightFootDown(bool down) {
        // Sensors
        sensors.fsr.right.fsr1 = down ? 1 : 0;
        sensors.fsr.right.fsr2 = down ? 1 : 0;
        sensors.fsr.right.fsr3 = down ? 1 : 0;
        sensors.fsr.right.fsr4 = down ? 1 : 0;

        // Centre
        sensors.fsr.right.centre_x = down ? 1 : std::numeric_limits<double>::quiet_NaN();
        sensors.fsr.right.centre_y = down ? 1 : std::numeric_limits<double>::quiet_NaN();
    }

    void HardwareSimulator::setLeftFootDown(bool down) {
        // Sensors
        sensors.fsr.left.fsr1 = down ? 1 : 0;
        sensors.fsr.left.fsr2 = down ? 1 : 0;
        sensors.fsr.left.fsr3 = down ? 1 : 0;
        sensors.fsr.left.fsr4 = down ? 1 : 0;

        // Centre
        sensors.fsr.left.centre_x = down ? 1 : std::numeric_limits<double>::quiet_NaN();
        sensors.fsr.left.centre_y = down ? 1 : std::numeric_limits<double>::quiet_NaN();
    }
}  // namespace module::platform
