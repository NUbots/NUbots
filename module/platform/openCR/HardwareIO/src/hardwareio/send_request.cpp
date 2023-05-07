#include <fmt/format.h>

#include "Convert.hpp"
#include "HardwareIO.hpp"

namespace module::platform::openCR {

    using message::platform::RawSensors;

    void HardwareIO::send_servo_request() {
        // Write out servo data
        // SYNC_WRITE (write the same memory addresses on all devices)
        // We need to do 2 sync writes here.
        // We always write to all servos if at least one of them is dirty

        const bool servos_dirty = std::any_of(servoStates.cbegin(),
                                              servoStates.cend(),
                                              [](const ServoState& servo) -> bool { return servo.dirty; });
        if (servos_dirty) {

            // log<NUClear::DEBUG>("Servos dirty, constructing SyncWrite");

            // Write data is split into two components
            std::array<dynamixel::v2::SyncWriteData<DynamixelServoWriteDataPart1>, 20> data1;
            std::array<dynamixel::v2::SyncWriteData<DynamixelServoWriteDataPart2>, 20> data2;

            for (uint i = 0; i < servoStates.size(); ++i) {
                // Servo ID is sequential, but not 0-indexed
                data1[i].id = nugus.servo_ids()[i];
                data2[i].id = nugus.servo_ids()[i];

                // Clear our dirty flag
                servoStates[i].dirty = false;

                // If our torque should be disabled then we disable our torque
                data1[i].data.torqueEnable =
                    uint8_t(servoStates[i].torque != 0 && !std::isnan(servoStates[i].goalPosition));

                // log<NUClear::DEBUG>(fmt::format(
                //     "Servo ID {}:\tservo state torque = {}\t data send torque = {}\tgoal position is NaN = {}",
                //     i + 1,
                //     servoStates[i].torqueEnabled,
                //     data1[i].data.torqueEnable,
                //     std::isnan(servoStates[i].goalPosition)));

                // Pack our data
                data1[i].data.velocityIGain = convert::IGain(servoStates[i].velocityIGain);
                data1[i].data.velocityPGain = convert::PGain(servoStates[i].velocityPGain);
                data1[i].data.positionDGain = convert::DGain(servoStates[i].positionDGain);
                data1[i].data.positionIGain = convert::IGain(servoStates[i].positionIGain);
                data1[i].data.positionPGain = convert::PGain(servoStates[i].positionPGain);

                data2[i].data.feedforward1stGain  = convert::FFGain(servoStates[i].feedforward1stGain);
                data2[i].data.feedforward2ndGain  = convert::FFGain(servoStates[i].feedforward2ndGain);
                data2[i].data.goalPWM             = convert::PWM(servoStates[i].goalPWM);
                data2[i].data.goalCurrent         = convert::current(servoStates[i].goalCurrent);
                data2[i].data.goalVelocity        = convert::velocity(servoStates[i].goalVelocity);
                data2[i].data.profileAcceleration = convert::FFGain(servoStates[i].profileAcceleration);
                data2[i].data.profileVelocity     = convert::FFGain(servoStates[i].profileVelocity);
                data2[i].data.goalPosition =
                    convert::position(i, servoStates[i].goalPosition, nugus.servo_direction, nugus.servo_offset);
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
        for (auto& id : nugus.servo_ids()) {
            packet_queue[id].push_back(PacketTypes::SERVO_DATA);
        }
        opencr.write(dynamixel::v2::SyncReadCommand<20>(uint16_t(AddressBook::SERVO_READ),
                                                        sizeof(DynamixelServoReadData),
                                                        nugus.servo_ids()));


        // Our final sensor output
        emit(std::make_unique<RawSensors>(construct_sensors()));
    }

    void HardwareIO::send_opencr_request() {
        // Write out OpenCR data
        if (opencrState.dirty) {
            // Clear the dirty flag
            opencrState.dirty = false;

            // Pack our data
            OpenCRWriteData data;
            data.led = opencrState.ledPanel.led4 ? 0x04 : 0x00;
            data.led |= opencrState.ledPanel.led3 ? 0x02 : 0x00;
            data.led |= opencrState.ledPanel.led2 ? 0x01 : 0x00;
            data.rgbLED = (uint8_t(0x000000FF & opencrState.headLED.RGB) & 0x1F) << 10;         // R
            data.rgbLED |= ((uint8_t(0x0000FF00 & opencrState.headLED.RGB) >> 8) & 0x1F) << 5;  // G
            data.rgbLED |= (uint8_t(0x00FF0000 & opencrState.headLED.RGB) >> 16) & 0x1F;        // B
            data.buzzer = opencrState.buzzer;

            // Write our data
            opencr.write(dynamixel::v2::WriteCommand<OpenCRWriteData>(uint8_t(NUgus::ID::OPENCR),
                                                                      uint16_t(OpenCR::Address::LED),
                                                                      data));
        }

        // Get OpenCR data
        // READ (only reading from a single device here)
        packet_queue[uint8_t(NUgus::ID::OPENCR)].push_back(PacketTypes::OPENCR_DATA);
        opencr.write(dynamixel::v2::ReadCommand(uint8_t(NUgus::ID::OPENCR),
                                                uint16_t(OpenCR::Address::LED),
                                                sizeof(OpenCRReadData)));
    }

    /**
     * Not really sure if any of this is correct, just one big massive guess
     */
    // Get FSR data
    // SYNC_READ from both FSRs
    // packet_queue[uint8_t(NUgus::ID::R_FSR)].push_back(PacketTypes::FSR_DATA);
    // packet_queue[uint8_t(NUgus::ID::L_FSR)].push_back(PacketTypes::FSR_DATA);
    // opencr.write(dynamixel::v2::SyncReadCommand<2>(uint16_t(AddressBook::FSR_READ),
    //                                                sizeof(FSRReadData),
    //                                                nugus.fsr_ids()));

}  // namespace module::platform::openCR