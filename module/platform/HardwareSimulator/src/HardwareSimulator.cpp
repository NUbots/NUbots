
/*
 * MIT License
 *
 * Copyright (c) 2014 NUbots
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

#include "HardwareSimulator.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <mutex>

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"
#include "message/input/Sensors.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/input/FrameID.hpp"
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

    using utility::input::FrameID;
    using utility::input::ServoID;
    using utility::support::Expression;

    HardwareSimulator::HardwareSimulator(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        /*
         Subcontroller Data
         */

        // Read our Error code
        sensors.subcontroller_error = 0;

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
        sensors.fsr.right.hardware_error = 0;

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
        sensors.fsr.left.hardware_error = 0;

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
            RawSensors::Servo& servo = utility::platform::get_raw_servo(i, sensors);

            // Error code
            servo.hardware_error = 0;

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
                    Eigen::Isometry3d Hf_rt(previousSensors->Htx[FrameID::R_ANKLE_ROLL]);
                    Eigen::Isometry3d Hf_lt(previousSensors->Htx[FrameID::L_ANKLE_ROLL]);
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
                    auto& servo       = utility::platform::get_raw_servo(i, sensors);
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
                    utility::platform::get_raw_servo(command.id, sensors).present_position);
                NUClear::clock::duration duration = command.time - NUClear::clock::now();

                float speed = 0.0f;
                if (duration.count() > 0) {
                    speed = diff / (double(duration.count()) / double(NUClear::clock::period::den));
                }

                // Set our variables
                auto& servo            = utility::platform::get_raw_servo(command.id, sensors);
                servo.profile_velocity = speed;
                servo.goal_position    = utility::math::angle::normalizeAngle(command.position);
            }
        });

        on<Trigger<ServoTarget>>().then([this](const ServoTarget& command) {
            auto commandList = std::make_unique<ServoTargets>();
            commandList->targets.push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::INLINE>(commandList);
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
