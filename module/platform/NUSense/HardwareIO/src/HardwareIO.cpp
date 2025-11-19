/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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

#include "NUSenseParser.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/ServoOffsets.hpp"
#include "message/actuation/ServoTarget.hpp"
#include "message/platform/NUSenseData.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/reflection.hpp"

#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"

namespace module::platform::NUSense {

    using extension::Configuration;
    using message::actuation::ServoOffsets;
    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;
    using message::actuation::SubcontrollerServoTarget;
    using message::actuation::SubcontrollerServoTargets;
    using message::platform::NUSense;
    using message::platform::NUSenseHandshake;
    using message::platform::RawSensors;
    using message::platform::ServoIDStates;
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

        on<Configuration>("NUSense.yaml").then([this](const Configuration& config) {
            this->log_level    = config["log_level"].as<NUClear::LogLevel>();
            std::string device = config["nusense"]["device"].as<std::string>();
            int baud           = config["nusense"]["baud"].as<int>();

            // Tell the stream reactor to connect to the device
            emit(std::make_unique<ConnectSerial>(std::move(device), baud));
        });

        on<PostConnect>().then("Sending handshake message to NUSense", [this] {
            // Parse handshake packet
            auto handshake = NUSenseHandshake();
            handshake.msg  = "";

            switch (service_state) {
                case ServiceState::INIT:
                    handshake.msg  = "NUC Handshake req";
                    handshake.type = false;
                    break;
                case ServiceState::IN_SERVICE:
                    handshake.msg  = "NUC Reconnect req";
                    handshake.type = true;
                    break;
            }

            for (size_t i = 0; i < cfg.servo_configurations.size(); ++i) {
                handshake.servo_configs.emplace_back(cfg.servo_configurations[i].direction,
                                                     cfg.servo_configurations[i].offset);
            }

            // Send to stream reactor
            send_packet(handshake);
            log<INFO>("Handshake message sent to NUSense, waiting for response...");
            log<INFO>(fmt::format("Type, {}", handshake.type));
        });

        // If this triggers, then that means that NUSense has acknowledged the NUC's message and the watchdog can be
        // unbound since it should not be needed after this point
        on<Trigger<NUSenseHandshake>>().then([this](const NUSenseHandshake& handshake) {
            // Unbind the handshake watchdog reaction here since it should not be needed anymore
            servo_targets_catcher.enable();
            service_state = ServiceState::IN_SERVICE;

            log<INFO>("Processing of ServoTargets enabled.");
            log<INFO>("ACK rx from NUSense: ", handshake.msg);
        });

        // Log Servo ID states
        on<Trigger<ServoIDStates>>().then("Servo ID states from NUSense", [this](const ServoIDStates& states) {
            log<INFO>("Received ServoIDStates from NUSense:");
            bool id_fault_detected = false;
            for (const auto& state : states.servo_id_states) {
                std::string state_str;
                switch (state.state.value) {
                    case ServoIDStates::IDState::PRESENT: state_str = "PRESENT"; break;
                    case ServoIDStates::IDState::MISSING:
                        state_str = "MISSING";
                        // Exclude ID 0 since it corresponds to NO_ID
                        id_fault_detected = state.id > 0 ? true : false;
                        break;
                    case ServoIDStates::IDState::DUPLICATE:
                        state_str = "DUPLICATE";
                        // Exclude ID 0 since it corresponds to NO_ID
                        id_fault_detected = state.id > 0 ? true : false;
                        break;
                }

                log<INFO>(fmt::format("Servo ID: {} {} State: {}",
                                      state.id,
                                      nugus.device_name(static_cast<NUgus::ID>(state.id)),
                                      state_str));
            }

            if (id_fault_detected) {
                log<ERROR>("Duplicate or missing servo IDs detected! (Excluding NO_ID)");
                exit(1);
            }
        });

        // Emit any messages sent by the device to the rest of the system
        on<Trigger<NUSenseFrame>>().then("From NUSense", [this](const NUSenseFrame& packet) {
            message::reflection::from_hash<EmitReflector>(packet.hash)->emit(powerplant, packet);
        });

        // Handle successfully decoded NUSense data
        // Single is added to prevent a seg fault, todo investigate further
        on<Trigger<NUSense>, With<ServoOffsets>, Single>().then(
            [this](const NUSense& data, const ServoOffsets& offsets) {
                // Will contain data from the NUSense to emit to the rest of the system
                auto sensors = std::make_unique<RawSensors>();

                // Timestamp when this message was created because the incoming packet doesn't have a timestamp
                sensors->timestamp = NUClear::clock::now();

                // Subcontroller data
                sensors->subcontroller_error = 0;  // TODO (NUSense people) not yet implemented
                sensors->led_panel           = 0;  // TODO (NUSense people) not yet implemented
                sensors->head_led            = 0;  // TODO (NUSense people) not yet implemented
                sensors->eye_led             = 0;  // TODO (NUSense people) not yet implemented
                sensors->buttons.left        = data.buttons.left;
                sensors->buttons.middle      = data.buttons.middle;

                // Set IMU
                sensors->accelerometer = Eigen::Vector3f(data.imu.accel.x, data.imu.accel.y, data.imu.accel.z);
                sensors->gyroscope     = Eigen::Vector3f(data.imu.gyro.x, data.imu.gyro.y, data.imu.gyro.z);

                // Battery data
                sensors->battery = 0;  // not yet implemented

                // Servo data
                for (const auto& [key, val] : data.servo_map) {
                    // Get a reference to the servo we are populating
                    RawSensors::Servo& servo = utility::platform::get_raw_servo(val.id - 1, *sensors);
                    // Fill all servo values from the reference
                    servo.hardware_error        = val.hardware_error;
                    servo.torque_enabled        = val.torque_enabled;
                    servo.velocity_i_gain       = 0;  // Not present in NUSense message
                    servo.velocity_p_gain       = 0;  // Not present in NUSense message
                    servo.position_d_gain       = 0;  // Not present in NUSense message
                    servo.position_i_gain       = 0;  // Not present in NUSense message
                    servo.position_p_gain       = 0;  // Not present in NUSense message
                    servo.feed_forward_1st_Gain = 0;  // Not present in NUSense message
                    servo.feed_forward_2nd_Gain = 0;  // Not present in NUSense message
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
                    servo.profile_acceleration  = 0;  // Not present in NUSense message
                    servo.profile_velocity      = 0;  // Not present in NUSense message

                    // Log any errors and timeouts from the servo.
                    if (val.packet_counts.packet_errors != 0) {
                        log<WARN>(fmt::format("{} packet-error(s) from ID {} ({})",
                                              val.packet_counts.packet_errors,
                                              val.id,
                                              nugus.device_name(static_cast<NUgus::ID>(val.id))));
                    }
                    if (val.packet_counts.crc_errors != 0) {
                        // For now, the CRC is set to debug until terminating-resistors are gotten since there are many
                        // when the robot is walking.
                        log<DEBUG>(fmt::format("{} CRC-error(s) from ID {} ({})",
                                               val.packet_counts.crc_errors,
                                               val.id,
                                               nugus.device_name(static_cast<NUgus::ID>(val.id))));
                    }
                    if (val.packet_counts.timeouts != 0) {
                        log<WARN>(fmt::format("{} dropped packet(s) from ID {} ({})",
                                              val.packet_counts.timeouts,
                                              val.id,
                                              nugus.device_name(static_cast<NUgus::ID>(val.id))));
                    }

                    // Add the offsets and switch the direction.
                    servo.present_position *= offsets.offsets[val.id - 1].direction;
                    servo.present_position += offsets.offsets[val.id - 1].offset;
                    servo.goal_position *= offsets.offsets[val.id - 1].direction;
                    servo.goal_position += offsets.offsets[val.id - 1].offset;
                }

                log<TRACE>(
                    fmt::format("\nIMU Data\n"
                                "\tAccel(xyz): {} - {} - {}\n"
                                "\t Gyro(xyz): {} - {} - {}\n ",
                                data.imu.accel.x,
                                data.imu.accel.y,
                                data.imu.accel.z,
                                data.imu.gyro.x,
                                data.imu.gyro.y,
                                data.imu.gyro.z));

                log<TRACE>("Logging servo states...");

                for (const auto& [key, val] : data.servo_map) {
                    log<TRACE>(fmt::format("      key: {}", key));

                    log<TRACE>(fmt::format("       id: {}", val.id));
                    log<TRACE>(fmt::format("   hw_err: {}", val.hardware_error));
                    log<TRACE>(fmt::format("torque_en: {}", val.torque_enabled));
                    log<TRACE>(fmt::format("     ppwm: {}", val.present_pwm));
                    log<TRACE>(fmt::format("    pcurr: {}", val.present_current));
                    log<TRACE>(fmt::format("    pvelo: {}", val.present_velocity));
                    log<TRACE>(fmt::format("     ppos: {}", val.present_position));
                    log<TRACE>(fmt::format("     gpwm: {}", val.goal_pwm));
                    log<TRACE>(fmt::format("    gcurr: {}", val.goal_current));
                    log<TRACE>(fmt::format("    gvelo: {}", val.goal_velocity));
                    log<TRACE>(fmt::format("     gpos: {}", val.goal_position));
                    log<TRACE>(fmt::format("  voltage: {}", val.voltage));
                    log<TRACE>(fmt::format("     temp: {}", val.temperature));
                }

                // Emit the raw sensor data
                emit(sensors);
            });

        on<Trigger<ServoTarget>>().then([this](const ServoTarget& command) {
            auto command_list = std::make_unique<ServoTargets>();
            command_list->targets.push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::INLINE>(std::move(command_list));
        });

        // Sync is used because uart write is a shared resource
        servo_targets_catcher = on<Trigger<ServoTargets>, With<ServoOffsets>, With<NUSense>, Sync<HardwareIO>>()
                                    .then([this](const ServoTargets& commands, const ServoOffsets& offsets) {
                                        // Copy the data into a new message so we can use a duration instead of a
                                        // timepoint and take the offsets and switch the direction.
                                        auto servo_targets = SubcontrollerServoTargets();

                                        // Change the timestamp in servo targets to the difference between the timestamp
                                        // and now Take away the offset and switch the direction if needed
                                        for (auto& target : commands.targets) {
                                            bool is_nan = std::isnan(target.position) || std::isnan(target.gain)
                                                          || std::isnan(target.torque);

                                            servo_targets.targets.emplace_back(
                                                target.time - NUClear::clock::now(),
                                                target.id,
                                                is_nan ? 0.0
                                                       : (target.position - offsets.offsets[target.id].offset)
                                                             * offsets.offsets[target.id].direction,
                                                is_nan ? 0.0 : target.gain,
                                                is_nan ? 0 : target.torque);
                                        }

                                        send_packet(servo_targets);
                                        log<DEBUG>("Sent a ServoTargets message to NUSense.");
                                    })
                                    .disable();
    }
#pragma GCC diagnostic pop

}  // namespace module::platform::NUSense
