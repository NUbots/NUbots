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
        const bool servos_dirty = std::any_of(servo_states.cbegin(),
                                              servo_states.cend(),
                                              [](const ServoState& servo) -> bool { return servo.dirty; });

        if (servos_dirty) {
            // Write data is split into two components
            dynamixel::v2::SyncWriteData<DynamixelServoWriteDataPart1> data1[20];
            dynamixel::v2::SyncWriteData<DynamixelServoWriteDataPart2> data2[20];

            for (uint i = 0; i < servo_states.size(); ++i) {
                // Servo ID is sequential, but not 0-indexed
                data1[i].id = nugus.servo_ids()[i];
                data2[i].id = nugus.servo_ids()[i];

                // Clear our dirty flag
                servo_states[i].dirty = false;

                // If our torque should be disabled then we disable our torque
                data1[i].data.torque_enable =
                    uint8_t(servo_states[i].torque != 0 && !std::isnan(servo_states[i].goal_position));

                // Pack our data
                data1[i].data.velocity_i_gain = convert::i_gain(servo_states[i].velocity_i_gain);
                data1[i].data.velocity_p_gain = convert::p_gain(servo_states[i].velocity_p_gain);
                data1[i].data.position_d_gain = convert::d_gain(servo_states[i].position_d_gain);
                data1[i].data.position_i_gain = convert::i_gain(servo_states[i].position_i_gain);
                data1[i].data.position_p_gain = convert::p_gain(servo_states[i].position_p_gain);

                data2[i].data.feedforward_1st_gain = convert::ff_gain(servo_states[i].feedforward_1st_gain);
                data2[i].data.feedforward_2nd_gain = convert::ff_gain(servo_states[i].feedforward_2nd_gain);
                data2[i].data.goal_pwm             = convert::PWM(servo_states[i].goal_pwm);
                data2[i].data.goal_current         = convert::current(servo_states[i].goal_current);
                data2[i].data.goal_velocity        = convert::velocity(servo_states[i].goal_velocity);
                data2[i].data.profile_acceleration = convert::ff_gain(servo_states[i].profile_acceleration);
                data2[i].data.profile_velocity     = convert::profile_velocity(servo_states[i].profile_velocity);
                data2[i].data.goal_position =
                    convert::position(i, servo_states[i].goal_position, nugus.servo_direction, nugus.servo_offset);
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

            // Rewrite this in a cleaner way
            // Run through all of the servo states and if even one motor is above the temeperature
            // threshold, fill the buzzer field
            data.buzzer = std::any_of(
                servo_states.begin(),
                servo_states.cend(),
                [](const ServoState& servo) -> bool { return servo.temperature > 70.0; }
            ) ? 880u : 0u;

            // This line serves as a naive test
            data.buzzer = 880u;

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
