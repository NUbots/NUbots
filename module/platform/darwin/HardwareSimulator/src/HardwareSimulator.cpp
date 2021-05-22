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

#include "message/input/Sensors.hpp"
#include "message/motion/ServoTarget.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/angle.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::platform::darwin {

    using extension::Configuration;

    using message::input::Sensors;
    using message::motion::ServoTarget;
    using message::platform::RawSensors;

    using utility::input::ServoID;
    using utility::nusight::graph;
    using utility::support::Expression;

    void addNoise(std::unique_ptr<RawSensors>& sensors) {
        // TODO: Use a more standard c++ random generator.
        sensors->accelerometer.x += noise.accelerometer.x * centered_noise();
        sensors->accelerometer.y += noise.accelerometer.y * centered_noise();
        sensors->accelerometer.z += noise.accelerometer.z * centered_noise();

        sensors->gyroscope.x += noise.gyroscope.x * centered_noise();
        sensors->gyroscope.y += noise.gyroscope.y * centered_noise();
        sensors->gyroscope.z += noise.gyroscope.z * centered_noise();
    }

    HardwareSimulator::HardwareSimulator(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), sensors(), gyroQueue(), gyroQueueMutex(), noise() {

        /*
         CM740 Data
         */
        // Read our Error code
        sensors.platform_error_flags = 0;

        // LED Panel
        sensors.led_panel.led2 = 0;
        sensors.led_panel.led3 = 0;
        sensors.led_panel.led4 = 0;

        // Head LED
        sensors.head_led.RGB = 0;

        // Head LED
        sensors.eye_led.RGB = 0;

        // Buttons
        sensors.buttons.left   = 0;
        sensors.buttons.middle = 0;

        // Voltage (in volts)
        sensors.voltage = 0;

        // Gyroscope (in radians/second)
        sensors.gyroscope.x = 0;
        sensors.gyroscope.y = 0;
        sensors.gyroscope.z = 0;

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
            servo.d_gain = 0;
            servo.i_gain = 0;
            servo.p_gain = 0;

            // Torque
            servo.torque = 0;

            // Targets
            servo.goal_position = 0;
            servo.moving_speed  = float(M_PI_4);

            // Present Data
            servo.present_position = 0;
            servo.present_speed    = 0;
            servo.load             = 0;

            // Diagnostic Information
            servo.voltage     = 0;
            servo.temperature = 0;
        }

        on<Configuration>("DarwinHardwareSimulator.yaml")
            .then("Hardware Simulator Config", [this](const Configuration& config) {
                imu_drift_rate = config["imu_drift_rate"].as<float>();

                noise.accelerometer.x = config["noise"]["accelerometer"]["x"].as<float>();
                noise.accelerometer.y = config["noise"]["accelerometer"]["y"].as<float>();
                noise.accelerometer.z = config["noise"]["accelerometer"]["z"].as<float>();

                noise.gyroscope.x = config["noise"]["gyroscope"]["x"].as<float>();
                noise.gyroscope.y = config["noise"]["gyroscope"]["y"].as<float>();
                noise.gyroscope.z = config["noise"]["gyroscope"]["z"].as<float>();

                bodyTilt = config["body_tilt"].as<Expression>();
            });

        on<Trigger<RawSensors::Gyroscope>>().then("Receive Simulated Gyroscope",
                                                  [this](const RawSensors::Gyroscope& gyro) {
                                                      std::lock_guard<std::mutex> lock(gyroQueueMutex);
                                                      RawSensors::Gyroscope tmpGyro = gyro;
                                                      gyroQueue.push(tmpGyro);
                                                  });


        on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Optional<With<Sensors>>, Single>().then(
            [this](std::shared_ptr<const Sensors> previousSensors) {
                if (previousSensors) {
                    Eigen::Affine3d Hf_rt(previousSensors->Htx[ServoID::R_ANKLE_ROLL]);
                    Eigen::Affine3d Hf_lt(previousSensors->Htx[ServoID::L_ANKLE_ROLL]);
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
                    float movingSpeed = servo.moving_speed == 0 ? 0.1f : servo.moving_speed / float(UPDATE_FREQUENCY);
                    movingSpeed       = movingSpeed > 0.1 ? 0.1f : movingSpeed;


                    if (std::fabs(servo.present_position - servo.goal_position) < movingSpeed) {
                        servo.present_position = servo.goal_position;
                    }
                    else {
                        Eigen::Vector3f present(std::cos(servo.present_position),
                                                std::sin(servo.present_position),
                                                0.0);
                        Eigen::Vector3f goal(std::cos(servo.goal_position), std::sin(servo.goal_position), 0.0);

                        Eigen::Vector3f cross = present.cross(goal);
                        if (cross.z() > 0.0f) {
                            servo.present_position =
                                utility::math::angle::normalizeAngle(servo.present_position + movingSpeed);
                        }
                        else {
                            servo.present_position =
                                utility::math::angle::normalizeAngle(servo.present_position - movingSpeed);
                        }
                    }
                }

                // Gyro:
                // Note: This reaction is not (and should not be) synced with the
                // 'Receive Simulated Gyroscope' reaction above, so we can't
                // reliably query the size of the gyroQueue.
                Eigen::Vector3f sumGyro = Eigen::Vector3f::Zero();
                /* mutext scope */ {
                    std::lock_guard<std::mutex> lock(gyroQueueMutex);
                    while (!gyroQueue.empty()) {
                        RawSensors::Gyroscope g = gyroQueue.front();
                        sumGyro += Eigen::Vector3d(g.x, g.y, g.z);

                        std::lock_guard<std::mutex> lock(gyroQueueMutex);
                        gyroQueue.pop();
                    }
                }
                sumGyro                 = (sumGyro * UPDATE_FREQUENCY + Eigen::Vector3f(0.0f, 0.0f, imu_drift_rate));
                sumGyro.x()             = -sumGyro.x();
                sensors.gyroscope.x     = sumGyro.x();
                sensors.gyroscope.y     = sumGyro.y();
                sensors.gyroscope.z     = sumGyro.z();
                sensors.accelerometer.x = -9.8f * std::sin(bodyTilt);
                sensors.accelerometer.y = 0.0f;
                sensors.accelerometer.z = -9.8f * std::cos(bodyTilt);
                sensors.timestamp       = NUClear::clock::now();

                // Add some noise so that sensor fusion doesnt converge to a singularity
                auto sensors_message = std::make_unique<RawSensors>(sensors);
                addNoise(sensors_message);

                // Send our nicely computed sensor data out to the world
                emit(std::move(sensors_message));
            });

        // This trigger writes the servo positions to the hardware
        on<Trigger<std::vector<ServoTarget>>>().then([this](const std::vector<ServoTarget>& commands) {
            for (auto& command : commands) {

                // Calculate our moving speed
                const float diff = utility::math::angle::difference(
                    command.position,
                    utility::platform::getRawServo(command.id, sensors).present_position);
                NUClear::clock::duration duration = command.time - NUClear::clock::now();

                float speed = 0.0f;
                if (duration.count() > 0) {
                    speed = diff / (float(duration.count()) / float(NUClear::clock::period::den));
                }

                // Set our variables
                auto& servo         = utility::platform::getRawServo(command.id, sensors);
                servo.moving_speed  = speed;
                servo.goal_position = utility::math::angle::normalizeAngle(command.position);
            }
        });

        on<Trigger<ServoTarget>>().then([this](const ServoTarget command) {
            auto commandList = std::make_unique<std::vector<ServoTarget>>();
            commandList->push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::DIRECT>(std::move(commandList));
        });
    }

    float centered_noise() {
        return float(rand()) / float(RAND_MAX) - 0.5f;
    }

    void HardwareSimulator::setRightFootDown(bool down) {
        // Sensors
        sensors.fsr.right.fsr1 = down ? 1 : 0;
        sensors.fsr.right.fsr2 = down ? 1 : 0;
        sensors.fsr.right.fsr3 = down ? 1 : 0;
        sensors.fsr.right.fsr4 = down ? 1 : 0;

        // Set the knee loads to something huge to be foot down
        utility::platform::getRawServo(ServoID::R_KNEE, sensors).load = down ? 1.0 : -1.0;

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

        // Set the knee loads to something huge to be foot down
        utility::platform::getRawServo(ServoID::L_KNEE, sensors).load = down ? 1.0 : -1.0;

        // Centre
        sensors.fsr.left.centre_x = down ? 1 : std::numeric_limits<double>::quiet_NaN();
        sensors.fsr.left.centre_y = down ? 1 : std::numeric_limits<double>::quiet_NaN();
    }
}  // namespace module::platform::darwin
