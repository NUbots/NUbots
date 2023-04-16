#include "HardwareIO.hpp"

namespace module::platform::openCR {

    void HardwareIO::startup() {
        // Set the OpenCR to not return a status packet when written to (to allow consecutive writes)
        opencr.write(dynamixel::v2::WriteCommand<uint8_t>(uint8_t(NUgus::ID::OPENCR),
                                                          uint16_t(OpenCR::Address::STATUS_RETURN_LEVEL),
                                                          uint8_t(1)));

        // Set the OpenCRs delay time to 0 (it may not have been configured before)
        opencr.write(dynamixel::v2::WriteCommand<uint8_t>(uint8_t(NUgus::ID::OPENCR),
                                                          uint16_t(OpenCR::Address::RETURN_DELAY_TIME),
                                                          uint8_t(0)));

        // Find OpenCR firmware and model versions
        packet_queue[uint8_t(NUgus::ID::OPENCR)].push_back(PacketTypes::MODEL_INFORMATION);
        opencr.write(dynamixel::v2::ReadCommand(uint8_t(NUgus::ID::OPENCR),
                                                uint16_t(OpenCR::Address::MODEL_NUMBER_L),
                                                uint8_t(3)));

        // Enable power to the servos
        opencr.write(dynamixel::v2::WriteCommand<uint8_t>(uint8_t(NUgus::ID::OPENCR),
                                                          uint16_t(OpenCR::Address::DYNAMIXEL_POWER),
                                                          uint8_t(1)));

        // Wait about 300ms for the dynamixels to start up
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        // Set the dynamixels to not return a status packet when written to (to allow consecutive writes)
        std::array<dynamixel::v2::SyncWriteData<uint8_t>, 20> data;
        for (int i = 0; i < 20; ++i) {
            // ensure write command targets the ID (ID != i)
            data[i] = dynamixel::v2::SyncWriteData<uint8_t>(nugus.servo_ids()[i], 1);
        }

        opencr.write(
            dynamixel::v2::SyncWriteCommand<uint8_t, 20>(uint16_t(DynamixelServo::Address::STATUS_RETURN_LEVEL), data));

        // Now that the dynamixels should have started up, set their delay time to 0 (it may not have been
        // configured before)
        for (int i = 0; i < 20; ++i) {
            // ensure write command targets the ID (ID != i)
            data[i] = dynamixel::v2::SyncWriteData<uint8_t>(nugus.servo_ids()[i], 0);
        }

        opencr.write(
            dynamixel::v2::SyncWriteCommand<uint8_t, 20>(uint16_t(DynamixelServo::Address::RETURN_DELAY_TIME), data));

        // Set up indirect addressing for read addresses
        std::array<dynamixel::v2::SyncWriteData<std::array<uint16_t, 17>>, 20> read_data;

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
        std::array<dynamixel::v2::SyncWriteData<std::array<uint16_t, 11>>, 20> write_data1;
        std::array<dynamixel::v2::SyncWriteData<std::array<uint16_t, 24>>, 20> write_data2;

        for (int i = 0; i < 20; ++i) {
            write_data1[i] = dynamixel::v2::SyncWriteData<std::array<uint16_t, 11>>(
                nugus.servo_ids()[i],  // ensure write command targets the ID (ID != i)
                {uint16_t(DynamixelServo::Address::TORQUE_ENABLE),
                 uint16_t(DynamixelServo::Address::VELOCITY_I_GAIN_L),
                 uint16_t(DynamixelServo::Address::VELOCITY_I_GAIN_H),
                 uint16_t(DynamixelServo::Address::VELOCITY_P_GAIN_L),
                 uint16_t(DynamixelServo::Address::VELOCITY_P_GAIN_H),
                 uint16_t(DynamixelServo::Address::VELOCITY_D_GAIN_L),
                 uint16_t(DynamixelServo::Address::VELOCITY_D_GAIN_H),
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
    }
}  // namespace module::platform::openCR
