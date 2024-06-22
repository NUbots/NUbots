/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#include "HardwareIO.hpp"

#include <Eigen/Core>
#include <fmt/format.h>
#include <vector>

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"
#include "message/platform/NUSenseData.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/reflection.hpp"

#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"


namespace module::platform::NUSense {

    using extension::Configuration;
    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;
    using message::actuation::SubcontrollerServoTarget;
    using message::actuation::SubcontrollerServoTargets;
    using message::platform::NUSense;
    using message::platform::RawSensors;
    using utility::support::Expression;


    /// @brief Message reflector class that can be used to emit messages provided as NUSenseFrames to the rest of the
    /// system
    /// @tparam T The type of the message to emit
    template <typename T>
    struct EmitReflector;

    /// Virtual base class for the emit reflector
    template <>
    struct EmitReflector<void> {  // NOLINT(cppcoreguidelines-special-member-functions)
        virtual void emit(NUClear::PowerPlant& powerplant, const NUSenseFrame& frame) = 0;
        virtual ~EmitReflector()                                                      = default;
    };
    template <typename T>
    struct EmitReflector : public EmitReflector<void> {
        void emit(NUClear::PowerPlant& powerplant, const NUSenseFrame& frame) override {
            // Deserialise and emit
            powerplant.emit(std::make_unique<T>(NUClear::util::serialise::Serialise<T>::deserialise(frame.payload)));
        }
    };

    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment)
        : utility::reactor::StreamReactor<HardwareIO, NUSenseParser, 5>(std::move(environment)) {

        on<Configuration>("HardwareIO.yaml").then([this](const Configuration& config) {
            // Use configuration here from file HardwareIO.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            auto device     = config["nusense"]["device"].as<std::string>();
            auto baud       = config["nusense"]["baud"].as<int>();

            // Tell the stream reactor to connect to the device
            emit(std::make_unique<ConnectSerial>(device, baud));

            // Apply servo offsets
            for (size_t i = 0; i < config["servos"].config.size(); ++i) {
                nugus.servo_offset[i]    = config["servos"][i]["offset"].as<Expression>();
                nugus.servo_direction[i] = config["servos"][i]["direction"].as<Expression>();
            }
        });

        // Emit any messages sent by the device to the rest of the system
        on<Trigger<NUSenseFrame>>().then("From NUSense", [this](const NUSenseFrame& packet) {
            message::reflection::from_hash<EmitReflector>(packet.hash)->emit(powerplant, packet);
        });

        // Handle successfully decoded NUSense data
        on<Trigger<NUSense>>().then([this](const NUSense& data) {
            // Will contain data from the NUSense to emit to the rest of the system
            auto sensors = std::make_unique<RawSensors>();

            // Timestamp when this message was created because the incoming packet doesn't have a timestamp
            sensors->timestamp = NUClear::clock::now();

            // Subcontroller data
            sensors->subcontroller_error = 0;  // TODO (NUSense people) not yet implemented
            sensors->led_panel           = 0;  // TODO (NUSense people) not yet implemented
            sensors->head_led            = 0;  // TODO (NUSense people) not yet implemented
            sensors->eye_led             = 0;  // TODO (NUSense people) not yet implemented
            sensors->buttons             = 0;  // TODO (NUSense people) not yet implemented

            // Set IMU
            sensors->accelerometer = Eigen::Vector3f(data.imu.accel.x, data.imu.accel.y, data.imu.accel.z);
            sensors->gyroscope     = Eigen::Vector3f(data.imu.gyro.x, data.imu.gyro.y, data.imu.gyro.z);

            // Battery data
            sensors->battery = 0;  // not yet implemented

            // Servo data
            for (const auto& [key, val] : data.servo_map) {
                // Get a reference to the servo we are populating
                RawSensors::Servo& servo = utility::platform::get_raw_servo(val.id - 1, *sensors);
                // fill all servo values from the reference
                servo.hardware_error        = val.hardware_error;
                servo.torque_enabled        = val.torque_enabled;
                servo.velocity_i_gain       = 0;  // not present in NUSense message
                servo.velocity_p_gain       = 0;  // not present in NUSense message
                servo.position_d_gain       = 0;  // not present in NUSense message
                servo.position_i_gain       = 0;  // not present in NUSense message
                servo.position_p_gain       = 0;  // not present in NUSense message
                servo.feed_forward_1st_Gain = 0;  // not present in NUSense message
                servo.feed_forward_2nd_Gain = 0;  // not present in NUSense message
                servo.present_PWM           = val.present_pwm;
                servo.present_current       = val.present_current;
                servo.present_velocity      = val.present_velocity;
                servo.present_position      = val.present_position;
                servo.goal_PWM              = val.goal_pwm;
                servo.goal_current          = val.goal_current;
                servo.goal_velocity         = val.goal_velocity;
                servo.goal_position         = val.goal_position;
                servo.voltage               = val.voltage;
                servo.temperature           = val.temperature;
                servo.profile_acceleration  = 0;  // not present in NUSense message
                servo.profile_velocity      = 0;  // not present in NUSense message

                // Log any errors and timeouts from the servo.
                if (val.packet_counts.packet_errors != 0) {
                    log<NUClear::WARN>(fmt::format("{} packet-error(s) from ID {} ({})",
                                                   val.packet_counts.packet_errors,
                                                   val.id,
                                                   nugus.device_name(static_cast<NUgus::ID>(val.id))));
                }
                if (val.packet_counts.crc_errors != 0) {
                    // For now, the CRC is set to debug until terminating-resistors are gotten since there are many when
                    // the robot is walking.
                    log<NUClear::DEBUG>(fmt::format("{} CRC-error(s) from ID {} ({})",
                                                    val.packet_counts.crc_errors,
                                                    val.id,
                                                    nugus.device_name(static_cast<NUgus::ID>(val.id))));
                }
                if (val.packet_counts.timeouts != 0) {
                    log<NUClear::WARN>(fmt::format("{} dropped packet(s) from ID {} ({})",
                                                   val.packet_counts.timeouts,
                                                   val.id,
                                                   nugus.device_name(static_cast<NUgus::ID>(val.id))));
                }

                // Add the offsets and switch the direction.
                servo.present_position *= nugus.servo_direction[val.id - 1];
                servo.present_position += nugus.servo_offset[val.id - 1];
            }

            log<NUClear::TRACE>(
                fmt::format("\nIMU Data\n"
                            "\tAccel(xyz): {} - {} - {}\n"
                            "\t Gyro(xyz): {} - {} - {}\n ",
                            data.imu.accel.x,
                            data.imu.accel.y,
                            data.imu.accel.z,
                            data.imu.gyro.x,
                            data.imu.gyro.y,
                            data.imu.gyro.z));

            log<NUClear::TRACE>("Logging servo states...");

            for (const auto& [key, val] : data.servo_map) {
                log<NUClear::TRACE>(fmt::format("      key: {}", key));

                log<NUClear::TRACE>(fmt::format("       id: {}", val.id));
                log<NUClear::TRACE>(fmt::format("   hw_err: {}", val.hardware_error));
                log<NUClear::TRACE>(fmt::format("torque_en: {}", val.torque_enabled));
                log<NUClear::TRACE>(fmt::format("     ppwm: {}", val.present_pwm));
                log<NUClear::TRACE>(fmt::format("    pcurr: {}", val.present_current));
                log<NUClear::TRACE>(fmt::format("    pvelo: {}", val.present_velocity));
                log<NUClear::TRACE>(fmt::format("     ppos: {}", val.present_position));
                log<NUClear::TRACE>(fmt::format("     gpwm: {}", val.goal_pwm));
                log<NUClear::TRACE>(fmt::format("    gcurr: {}", val.goal_current));
                log<NUClear::TRACE>(fmt::format("    gvelo: {}", val.goal_velocity));
                log<NUClear::TRACE>(fmt::format("     gpos: {}", val.goal_position));
                log<NUClear::TRACE>(fmt::format("  voltage: {}", val.voltage));
                log<NUClear::TRACE>(fmt::format("     temp: {}", val.temperature));
            }

            // Emit the raw sensor data
            emit(std::move(sensors));
        });


        // Sync is used because uart write is a shared resource
        on<Trigger<ServoTargets>, Sync<HardwareIO>>().then([this](const ServoTargets& commands) {
            // Copy the data into a new message so we can use a duration instead of a timepoint
            // and take the offsets and switch the direction.
            auto servo_targets = SubcontrollerServoTargets();

            // Change the timestamp in servo targets to the difference between the timestamp and now
            // Take away the offset and switch the direction if needed
            for (auto& target : commands.targets) {
                servo_targets.targets.emplace_back(
                    target.time - NUClear::clock::now(),
                    target.id,
                    (target.position - nugus.servo_offset[target.id]) * nugus.servo_direction[target.id],
                    target.gain,
                    target.torque);
            }

            send_packet(servo_targets);
        });

        on<Trigger<ServoTarget>>().then([this](const ServoTarget& command) {
            auto command_list = std::make_unique<ServoTargets>();
            command_list->targets.push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::DIRECT>(std::move(command_list));
        });
    }

}  // namespace module::platform::NUSense
