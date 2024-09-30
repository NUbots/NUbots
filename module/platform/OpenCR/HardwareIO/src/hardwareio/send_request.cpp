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
#include <fmt/format.h>

#include "Convert.hpp"
#include "HardwareIO.hpp"

namespace module::platform::OpenCR {

    using message::platform::RawSensors;

    void HardwareIO::send_servo_request() {

        // Write out servo data
        // SYNC_WRITE (write the same memory addresses on all devices)
        // We need to do 2 sync writes here.
        // We always write to all servos if at least one of them is dirty
        const bool servos_dirty =
            std::any_of(servos.cbegin(), servos.cend(), [](const Servo& servo) -> bool { return servo.state.dirty; });

        if (servos_dirty) {
            // Write data is split into two components
            dynamixel::v2::SyncWriteData<DynamixelServoWriteDataPart1> data1[20];
            dynamixel::v2::SyncWriteData<DynamixelServoWriteDataPart2> data2[20];

            for (uint i = 0; i < servos.size(); ++i) {
                // Servo ID is sequential, but not 0-indexed
                data1[i].id = nugus.servo_ids()[i];
                data2[i].id = nugus.servo_ids()[i];

                // Clear our dirty flag
                servos[i].state.dirty = false;

                // If our torque should be disabled then we disable our torque
                data1[i].data.torque_enable =
                    uint8_t(servos[i].goal.torque_enabled != 0 && !std::isnan(servos[i].goal.goal_position));

                // Pack our data
                data1[i].data.velocity_p_gain = convert::p_gain(0.0);
                data1[i].data.velocity_i_gain = convert::i_gain(0.0);
                data1[i].data.position_p_gain = convert::p_gain(servos[i].goal.position_p_gain);
                data1[i].data.position_i_gain = convert::i_gain(servos[i].goal.position_i_gain);
                data1[i].data.position_d_gain = convert::d_gain(servos[i].goal.position_d_gain);

                data2[i].data.feedforward_1st_gain = convert::ff_gain(servos[i].goal.feedforward_1st_gain);
                data2[i].data.feedforward_2nd_gain = convert::ff_gain(servos[i].goal.feedforward_2nd_gain);
                data2[i].data.goal_pwm             = convert::PWM(servos[i].goal.goal_pwm);
                data2[i].data.goal_current         = convert::current(servos[i].goal.goal_current);
                data2[i].data.goal_velocity        = convert::velocity(servos[i].goal.goal_velocity);
                data2[i].data.profile_acceleration = convert::ff_gain(servos[i].goal.profile_acceleration);
                data2[i].data.profile_velocity     = convert::profile_velocity(servos[i].goal.profile_velocity);
                data2[i].data.goal_position =
                    convert::position(i, servos[i].goal.goal_position, nugus.servo_direction, nugus.servo_offset);
            }

            opencr.write(
                dynamixel::v2::SyncWriteCommand<DynamixelServoWriteDataPart1, 20>(uint16_t(AddressBook::SERVO_WRITE_1),
                                                                                  data1));
            opencr.write(
                dynamixel::v2::SyncWriteCommand<DynamixelServoWriteDataPart2, 20>(uint16_t(AddressBook::SERVO_WRITE_2),
                                                                                  data2));
        }

        // Get updated servo data
        // SYNC_READ (read the same memory addresses on all devices)
        for (const auto& id : nugus.servo_ids()) {
            packet_queue[NUgus::ID(id)].push_back(PacketTypes::SERVO_DATA);
        }
        opencr.write(dynamixel::v2::SyncReadCommand<20>(uint16_t(AddressBook::SERVO_READ),
                                                        sizeof(DynamixelServoReadData),
                                                        nugus.servo_ids()));


        // Our final sensor output
        emit(std::make_unique<RawSensors>(construct_sensors()));
    }

    void HardwareIO::send_opencr_request() {
        // Write out OpenCR data
        if (opencr_state.dirty) {
            // Clear the dirty flag
            opencr_state.dirty = false;

            // Pack our data
            OpenCRWriteData data;
            data.led = opencr_state.led_panel.led4 ? 0x04 : 0x00;
            data.led |= opencr_state.led_panel.led3 ? 0x02 : 0x00;
            data.led |= opencr_state.led_panel.led2 ? 0x01 : 0x00;
            data.rgb_led = (uint8_t(0x000000FF & opencr_state.head_led.RGB) & 0x1F) << 10;         // R
            data.rgb_led |= ((uint8_t(0x0000FF00 & opencr_state.head_led.RGB) >> 8) & 0x1F) << 5;  // G
            data.rgb_led |= (uint8_t(0x00FF0000 & opencr_state.head_led.RGB) >> 16) & 0x1F;        // B
            data.buzzer = opencr_state.buzzer;

            // Write our data
            opencr.write(dynamixel::v2::WriteCommand<OpenCRWriteData>(uint8_t(NUgus::ID::OPENCR),
                                                                      uint16_t(OpenCR::Address::LED),
                                                                      data));
        }

        // Get OpenCR data
        // READ (only reading from a single device here)
        packet_queue[NUgus::ID::OPENCR].push_back(PacketTypes::OPENCR_DATA);
        opencr.write(dynamixel::v2::ReadCommand(uint8_t(NUgus::ID::OPENCR),
                                                uint16_t(OpenCR::Address::LED),
                                                sizeof(OpenCRReadData)));
    }

}  // namespace module::platform::OpenCR
