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

    void HardwareIO::startup() {
        // first, disable the packet watchdog as we haven't sent anything yet and don't want it to trigger
        packet_watchdog.disable();

        // Set the OpenCR to not return a status packet when written to (to allow consecutive writes)
        opencr.write(dynamixel::v2::WriteCommand<uint8_t>(uint8_t(NUgus::ID::OPENCR),
                                                          uint16_t(OpenCR::Address::STATUS_RETURN_LEVEL),
                                                          uint8_t(1)));

        // Set the OpenCRs delay time to 0 (it may not have been configured before)
        opencr.write(dynamixel::v2::WriteCommand<uint8_t>(uint8_t(NUgus::ID::OPENCR),
                                                          uint16_t(OpenCR::Address::RETURN_DELAY_TIME),
                                                          uint8_t(0)));

        // We do not disable and enable the servo power on startup because doing so would make the robot collapse if it
        // were standing, possibly resulting in damage. Instead, we leave the servos powered on and if there are any
        // servo errors, they can be cleared by using the red button to power off the servos, or by turning off the
        // robot entirely.
        opencr.write(dynamixel::v2::WriteCommand<uint8_t>(uint8_t(NUgus::ID::OPENCR),
                                                          uint16_t(OpenCR::Address::DYNAMIXEL_POWER),
                                                          uint8_t(1)));

        // Wait about 300ms for the dynamixels to start up
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        // Set all dynamixels to not return a status packet when written to (to allow consecutive writes)
        dynamixel::v2::SyncWriteData<uint8_t> data[20];
        for (int i = 0; i < 20; ++i) {
            // ensure write command targets the ID (ID != i)
            data[i] = dynamixel::v2::SyncWriteData<uint8_t>(nugus.servo_ids()[i], 0);
        }
        opencr.write(
            dynamixel::v2::SyncWriteCommand<uint8_t, 20>(uint16_t(DynamixelServo::Address::STATUS_RETURN_LEVEL), data));

        // Now that all dynamixels should have started up, set their delay time to 0 (it may not have been
        // configured before)
        for (int i = 0; i < 20; ++i) {
            // ensure write command targets the ID (ID != i)
            data[i] = dynamixel::v2::SyncWriteData<uint8_t>(nugus.servo_ids()[i], 0);
        }
        opencr.write(
            dynamixel::v2::SyncWriteCommand<uint8_t, 20>(uint16_t(DynamixelServo::Address::RETURN_DELAY_TIME), data));

        // Set up the dynamixels to use time-based profile velocity control
        for (int i = 0; i < 20; ++i) {
            // ensure write command targets the ID (ID != i)
            data[i] = dynamixel::v2::SyncWriteData<uint8_t>(nugus.servo_ids()[i], 4);
        }
        opencr.write(dynamixel::v2::SyncWriteCommand<uint8_t, 20>(uint16_t(DynamixelServo::Address::DRIVE_MODE), data));


        // Set position limits for the servos
        dynamixel::v2::SyncWriteData<uint8_t> max_position_limits_data[20];
        dynamixel::v2::SyncWriteData<uint8_t> min_position_limits_data[20];
        for (int i = 0; i < 20; ++i) {
            // Ensure write command targets the ID (ID != i)

            const uint8_t id = nugus.servo_ids()[i];
            // Set the max rotation limits on all servos
            // max_position_limits_data[i] = dynamixel::v2::SyncWriteData<uint8_t>(
            //     id,
            //     convert::position(id, nugus.servo_max_position_limits[i], nugus.servo_direction,
            //     nugus.servo_offset));
            max_position_limits_data[i] = dynamixel::v2::SyncWriteData<uint8_t>(id, uint8_t(5));

            // **** DEBUG max_pos ****
            log<NUClear::DEBUG>(fmt::format(
                "MAX POS DEBUG - id:{} - limit:{}",
                id,
                convert::position(id, nugus.servo_max_position_limits[i], nugus.servo_direction, nugus.servo_offset)));

            // Set the min rotation limits on all servos
            // min_position_limits_data[i] = dynamixel::v2::SyncWriteData<uint8_t>(
            //     id,
            //     convert::position(id, nugus.servo_min_position_limits[i], nugus.servo_direction,
            //     nugus.servo_offset));
            min_position_limits_data[i] = dynamixel::v2::SyncWriteData<uint8_t>(id, uint8_t(5));

            // **** DEBUG min_pos ****
            log<NUClear::DEBUG>(fmt::format(
                "MIN POS DEBUG - id:{} - limit:{}",
                id,
                convert::position(id, nugus.servo_min_position_limits[i], nugus.servo_direction, nugus.servo_offset)));
        }
        // opencr.write(
        //     dynamixel::v2::SyncWriteCommand<uint8_t, 20>(uint16_t(DynamixelServo::Address::MAX_POSITION_LIMIT_L),
        //                                                  max_position_limits_data));
        // opencr.write(
        //     dynamixel::v2::SyncWriteCommand<uint8_t, 20>(uint16_t(DynamixelServo::Address::MIN_POSITION_LIMIT_L),
        //                                                  min_position_limits_data));

        // TEST - writing a single servo's position limit
        // opencr.write(dynamixel::v2::WriteCommand<uint8_t>(uint8_t(DynamixelServo::Address::MAX_POSITION_LIMIT_L),
        //                                                   uint16_t(nugus.servo_ids()[19]),
        //                                                   uint8_t(5)));

        // Set up indirect addressing for read addresses for each dynamixel
        dynamixel::v2::SyncWriteData<std::array<uint16_t, 17>> read_data[20];

        for (int i = 0; i < 20; ++i) {
            read_data[i] = dynamixel::v2::SyncWriteData<std::array<uint16_t, 17>>(
                nugus.servo_ids()[i],  // ensure write command targets the ID (ID != i)
                {uint16_t(DynamixelServo::Address::TORQUE_ENABLE),
                 uint16_t(DynamixelServo::Address::HARDWARE_ERROR_STATUS),
                 uint16_t(DynamixelServo::Address::PRESENT_PWM_L),
                 uint16_t(DynamixelServo::Address::PRESENT_PWM_H),
                 uint16_t(DynamixelServo::Address::PRESENT_CURRENT_L),
                 uint16_t(DynamixelServo::Address::PRESENT_CURRENT_H),
                 uint16_t(DynamixelServo::Address::PRESENT_VELOCITY_L),
                 uint16_t(DynamixelServo::Address::PRESENT_VELOCITY_2),
                 uint16_t(DynamixelServo::Address::PRESENT_VELOCITY_3),
                 uint16_t(DynamixelServo::Address::PRESENT_VELOCITY_H),
                 uint16_t(DynamixelServo::Address::PRESENT_POSITION_L),
                 uint16_t(DynamixelServo::Address::PRESENT_POSITION_2),
                 uint16_t(DynamixelServo::Address::PRESENT_POSITION_3),
                 uint16_t(DynamixelServo::Address::PRESENT_POSITION_H),
                 uint16_t(DynamixelServo::Address::PRESENT_INPUT_VOLTAGE_L),
                 uint16_t(DynamixelServo::Address::PRESENT_INPUT_VOLTAGE_H),
                 uint16_t(DynamixelServo::Address::PRESENT_TEMPERATURE)});
        }

        opencr.write(
            dynamixel::v2::SyncWriteCommand<std::array<uint16_t, 17>, 20>(uint16_t(AddressBook::SERVO_READ_ADDRESS),
                                                                          read_data));

        // Set up indirect addressing for write addresses
        dynamixel::v2::SyncWriteData<std::array<uint16_t, 11>> write_data1[20];
        dynamixel::v2::SyncWriteData<std::array<uint16_t, 24>> write_data2[20];

        for (int i = 0; i < 20; ++i) {
            write_data1[i] = dynamixel::v2::SyncWriteData<std::array<uint16_t, 11>>(
                nugus.servo_ids()[i],  // ensure write command targets the ID (ID != i)
                {uint16_t(DynamixelServo::Address::TORQUE_ENABLE),
                 uint16_t(DynamixelServo::Address::VELOCITY_I_GAIN_L),
                 uint16_t(DynamixelServo::Address::VELOCITY_I_GAIN_H),
                 uint16_t(DynamixelServo::Address::VELOCITY_P_GAIN_L),
                 uint16_t(DynamixelServo::Address::VELOCITY_P_GAIN_H),
                 uint16_t(DynamixelServo::Address::POSITION_D_GAIN_L),
                 uint16_t(DynamixelServo::Address::POSITION_D_GAIN_H),
                 uint16_t(DynamixelServo::Address::POSITION_I_GAIN_L),
                 uint16_t(DynamixelServo::Address::POSITION_I_GAIN_H),
                 uint16_t(DynamixelServo::Address::POSITION_P_GAIN_L),
                 uint16_t(DynamixelServo::Address::POSITION_P_GAIN_H)});

            write_data2[i] = dynamixel::v2::SyncWriteData<std::array<uint16_t, 24>>(
                nugus.servo_ids()[i],  // ensure write command targets the ID (ID != i)
                {uint16_t(DynamixelServo::Address::FEEDFORWARD_1ST_GAIN_L),
                 uint16_t(DynamixelServo::Address::FEEDFORWARD_1ST_GAIN_H),
                 uint16_t(DynamixelServo::Address::FEEDFORWARD_2ND_GAIN_L),
                 uint16_t(DynamixelServo::Address::FEEDFORWARD_2ND_GAIN_H),
                 uint16_t(DynamixelServo::Address::GOAL_PWM_L),
                 uint16_t(DynamixelServo::Address::GOAL_PWM_H),
                 uint16_t(DynamixelServo::Address::GOAL_CURRENT_L),
                 uint16_t(DynamixelServo::Address::GOAL_CURRENT_H),
                 uint16_t(DynamixelServo::Address::GOAL_VELOCITY_L),
                 uint16_t(DynamixelServo::Address::GOAL_VELOCITY_2),
                 uint16_t(DynamixelServo::Address::GOAL_VELOCITY_3),
                 uint16_t(DynamixelServo::Address::GOAL_VELOCITY_H),
                 uint16_t(DynamixelServo::Address::PROFILE_ACCELERATION_L),
                 uint16_t(DynamixelServo::Address::PROFILE_ACCELERATION_2),
                 uint16_t(DynamixelServo::Address::PROFILE_ACCELERATION_3),
                 uint16_t(DynamixelServo::Address::PROFILE_ACCELERATION_H),
                 uint16_t(DynamixelServo::Address::PROFILE_VELOCITY_L),
                 uint16_t(DynamixelServo::Address::PROFILE_VELOCITY_2),
                 uint16_t(DynamixelServo::Address::PROFILE_VELOCITY_3),
                 uint16_t(DynamixelServo::Address::PROFILE_VELOCITY_H),
                 uint16_t(DynamixelServo::Address::GOAL_POSITION_L),
                 uint16_t(DynamixelServo::Address::GOAL_POSITION_2),
                 uint16_t(DynamixelServo::Address::GOAL_POSITION_3),
                 uint16_t(DynamixelServo::Address::GOAL_POSITION_H)});
        }

        opencr.write(
            dynamixel::v2::SyncWriteCommand<std::array<uint16_t, 11>, 20>(uint16_t(AddressBook::SERVO_WRITE_ADDRESS_1),
                                                                          write_data1));
        opencr.write(
            dynamixel::v2::SyncWriteCommand<std::array<uint16_t, 24>, 20>(uint16_t(AddressBook::SERVO_WRITE_ADDRESS_2),
                                                                          write_data2));

        // Loop over all servo states and set the servos to uninitialised so we can read their state and update our
        // internal goal state
        for (auto& servo_state : servo_states) {
            servo_state.initialised = false;
        }

        // Now, enable the packet watchdog before we send anything.
        packet_watchdog.enable();

        // Find OpenCR firmware and model versions
        // This has to be called last, as we need to wait for the response packet before starting
        packet_queue[NUgus::ID::OPENCR].push_back(PacketTypes::MODEL_INFORMATION);
        opencr.write(dynamixel::v2::ReadCommand(uint8_t(NUgus::ID::OPENCR),
                                                uint16_t(OpenCR::Address::MODEL_NUMBER_L),
                                                uint8_t(3)));
    }
}  // namespace module::platform::OpenCR
