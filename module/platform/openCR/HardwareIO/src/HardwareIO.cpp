#include "HardwareIO.hpp"

#include <fmt/format.h>

#include "Convert.hpp"
#include "dynamixel/v2/Dynamixel.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/comparison.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::platform::openCR {

    using extension::Configuration;
    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;
    using message::platform::RawSensors;
    using message::platform::StatusReturn;
    using utility::support::Expression;

    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), opencr(), nugus(), byte_wait(0), packet_wait(0), packet_queue() {

        on<Configuration>("HardwareIO_OpenCR.yaml").then([this](const Configuration& config) {
            // Make sure OpenCR is operating at the correct baud rate (based on config params)
            if (opencr.connected()) {
                opencr.close();
            }

            opencr.open(config["opencr"]["device"], config["opencr"]["baud"]);
            byte_wait   = config["opencr"]["byte_wait"];
            packet_wait = config["opencr"]["packet_wait"];

            // Initialise packet_queue map
            packet_queue[uint8_t(NUgus::ID::OPENCR)] = std::vector<PacketTypes>();
            for (int i = 0; i < 20; ++i) {
                packet_queue[i] = std::vector<PacketTypes>();
            }

            for (size_t i = 0; i < config["servos"].config.size(); ++i) {
                nugus.servo_offset[i]    = config["servos"][i]["offset"].as<Expression>();
                nugus.servo_direction[i] = config["servos"][i]["direction"].as<Expression>();
                servoState[i].simulated  = config["servos"][i]["simulated"].as<bool>();
            }
        });

        on<Startup>().then("HardwareIO Startup", [this] {
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
                data[i] = dynamixel::v2::SyncWriteData<uint8_t>(i, 1);
            }

            opencr.write(
                dynamixel::v2::SyncWriteCommand<uint8_t, 20>(uint16_t(MX64::Address::STATUS_RETURN_LEVEL), data));

            // Now that the dynamixels should have started up, set their delay time to 0 (it may not have been
            // configured before)
            for (int i = 0; i < 20; ++i) {
                data[i] = dynamixel::v2::SyncWriteData<uint8_t>(i, 0);
            }

            opencr.write(
                dynamixel::v2::SyncWriteCommand<uint8_t, 20>(uint16_t(MX64::Address::RETURN_DELAY_TIME), data));

            // Set up indirect addressing for read addresses
            std::array<dynamixel::v2::SyncWriteData<std::array<uint16_t, 17>>, 20> read_data;

            for (int i = 0; i < 20; ++i) {
                read_data[i] = dynamixel::v2::SyncWriteData<std::array<uint16_t, 17>>(
                    i,
                    {uint16_t(MX64::Address::TORQUE_ENABLE),
                     uint16_t(MX64::Address::HARDWARE_ERROR_STATUS),
                     uint16_t(MX64::Address::PRESENT_PWM_L),
                     uint16_t(MX64::Address::PRESENT_PWM_H),
                     uint16_t(MX64::Address::PRESENT_CURRENT_L),
                     uint16_t(MX64::Address::PRESENT_CURRENT_H),
                     uint16_t(MX64::Address::PRESENT_VELOCITY_L),
                     uint16_t(MX64::Address::PRESENT_VELOCITY_2),
                     uint16_t(MX64::Address::PRESENT_VELOCITY_3),
                     uint16_t(MX64::Address::PRESENT_VELOCITY_H),
                     uint16_t(MX64::Address::PRESENT_POSITION_L),
                     uint16_t(MX64::Address::PRESENT_POSITION_2),
                     uint16_t(MX64::Address::PRESENT_POSITION_3),
                     uint16_t(MX64::Address::PRESENT_POSITION_H),
                     uint16_t(MX64::Address::PRESENT_INPUT_VOLTAGE_L),
                     uint16_t(MX64::Address::PRESENT_INPUT_VOLTAGE_H),
                     uint16_t(MX64::Address::PRESENT_TEMPERATURE)});
            }

            opencr.write(dynamixel::v2::SyncWriteCommand<std::array<uint16_t, 17>, 20>(
                uint16_t(DynamixelIndirect::SERVO_READ_ADDRESS),
                read_data));

            // Set up indirect addressing for write addresses
            std::array<dynamixel::v2::SyncWriteData<std::array<uint16_t, 11>>, 20> write_data1;
            std::array<dynamixel::v2::SyncWriteData<std::array<uint16_t, 24>>, 20> write_data2;

            for (int i = 0; i < 20; ++i) {
                write_data1[i] = dynamixel::v2::SyncWriteData<std::array<uint16_t, 11>>(
                    i,
                    {uint16_t(MX64::Address::TORQUE_ENABLE),
                     uint16_t(MX64::Address::VELOCITY_I_GAIN_L),
                     uint16_t(MX64::Address::VELOCITY_I_GAIN_H),
                     uint16_t(MX64::Address::VELOCITY_P_GAIN_L),
                     uint16_t(MX64::Address::VELOCITY_P_GAIN_H),
                     uint16_t(MX64::Address::VELOCITY_D_GAIN_L),
                     uint16_t(MX64::Address::VELOCITY_D_GAIN_H),
                     uint16_t(MX64::Address::POSITION_I_GAIN_L),
                     uint16_t(MX64::Address::POSITION_I_GAIN_H),
                     uint16_t(MX64::Address::POSITION_P_GAIN_L),
                     uint16_t(MX64::Address::POSITION_P_GAIN_H)});

                write_data2[i] = dynamixel::v2::SyncWriteData<std::array<uint16_t, 24>>(
                    i,
                    {uint16_t(MX64::Address::FEEDFORWARD_1ST_GAIN_L),
                     uint16_t(MX64::Address::FEEDFORWARD_1ST_GAIN_H),
                     uint16_t(MX64::Address::FEEDFORWARD_2ND_GAIN_L),
                     uint16_t(MX64::Address::FEEDFORWARD_2ND_GAIN_H),
                     uint16_t(MX64::Address::GOAL_PWM_L),
                     uint16_t(MX64::Address::GOAL_PWM_H),
                     uint16_t(MX64::Address::GOAL_CURRENT_L),
                     uint16_t(MX64::Address::GOAL_CURRENT_H),
                     uint16_t(MX64::Address::GOAL_VELOCITY_L),
                     uint16_t(MX64::Address::GOAL_VELOCITY_2),
                     uint16_t(MX64::Address::GOAL_VELOCITY_3),
                     uint16_t(MX64::Address::GOAL_VELOCITY_H),
                     uint16_t(MX64::Address::PROFILE_ACCELERATION_L),
                     uint16_t(MX64::Address::PROFILE_ACCELERATION_2),
                     uint16_t(MX64::Address::PROFILE_ACCELERATION_3),
                     uint16_t(MX64::Address::PROFILE_ACCELERATION_H),
                     uint16_t(MX64::Address::PROFILE_VELOCITY_L),
                     uint16_t(MX64::Address::PROFILE_VELOCITY_2),
                     uint16_t(MX64::Address::PROFILE_VELOCITY_3),
                     uint16_t(MX64::Address::PROFILE_VELOCITY_H),
                     uint16_t(MX64::Address::GOAL_POSITION_L),
                     uint16_t(MX64::Address::GOAL_POSITION_2),
                     uint16_t(MX64::Address::GOAL_POSITION_3),
                     uint16_t(MX64::Address::GOAL_POSITION_H)});
            }

            opencr.write(dynamixel::v2::SyncWriteCommand<std::array<uint16_t, 11>, 20>(
                uint16_t(DynamixelIndirect::SERVO_WRITE_ADDRESS_1),
                write_data1));
            opencr.write(dynamixel::v2::SyncWriteCommand<std::array<uint16_t, 24>, 20>(
                uint16_t(DynamixelIndirect::SERVO_WRITE_ADDRESS_2),
                write_data2));
        });

        on<Shutdown>().then("HardwareIO Startup", [this] {
            // Close our connection to the OpenCR
            if (opencr.connected()) {
                opencr.close();
            }
        });

        // This trigger gets the sensor data from the sub controller
        on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Single, Priority::HIGH>().then("Hardware Loop", [this] {
            // Write out servo data
            // SYNC_WRITE (write the same memory addresses on all devices)
            // We need to do 2 sync writes here.
            // We always write to all servos if at least one of them is dirty
            const bool servos_dirty = std::any_of(servoState.cbegin(),
                                                  servoState.cend(),
                                                  [](const ServoState& servo) -> bool { return servo.dirty; });
            if (servos_dirty) {
                std::array<dynamixel::v2::SyncWriteData<DynamixelServoWriteDataPart1>, 20> data1;
                std::array<dynamixel::v2::SyncWriteData<DynamixelServoWriteDataPart2>, 20> data2;

                for (uint i = 0; i < servoState.size(); ++i) {
                    data1[i].id = i;
                    data2[i].id = i;

                    // Clear our dirty flag
                    servoState[i].dirty = false;

                    // If our torque should be disabled then we disable our torque
                    if (servoState[i].torqueEnabled
                        && (std::isnan(servoState[i].goalPosition) || servoState[i].goalCurrent == 0)) {
                        servoState[i].torqueEnabled = false;
                        data1[i].data.torqueEnable  = 0;
                    }
                    else {
                        // If our torque was disabled but is now enabled
                        if (!servoState[i].torqueEnabled && !std::isnan(servoState[i].goalPosition)
                            && servoState[i].goalCurrent != 0) {
                            servoState[i].torqueEnabled = true;
                            data1[i].data.torqueEnable  = 1;
                        }
                    }

                    // Pack our data
                    data1[i].data.velocityPGain = convert::PGain(servoState[i].velocityPGain);
                    data1[i].data.velocityIGain = convert::IGain(servoState[i].velocityIGain);
                    data1[i].data.velocityDGain = convert::DGain(servoState[i].velocityDGain);
                    data1[i].data.positionPGain = convert::PGain(servoState[i].positionPGain);
                    data1[i].data.positionIGain = convert::IGain(servoState[i].positionIGain);

                    data2[i].data.feedforward1stGain  = convert::FFGain(servoState[i].feedforward1stGain);
                    data2[i].data.feedforward2ndGain  = convert::FFGain(servoState[i].feedforward2ndGain);
                    data2[i].data.goalPWM             = convert::PWM(servoState[i].goalPWM);
                    data2[i].data.goalCurrent         = convert::current(servoState[i].goalCurrent);
                    data2[i].data.goalVelocity        = convert::velocity(servoState[i].goalVelocity);
                    data2[i].data.profileAcceleration = convert::FFGain(servoState[i].profileAcceleration);
                    data2[i].data.profileVelocity     = convert::FFGain(servoState[i].profileVelocity);
                    data2[i].data.goalPosition =
                        convert::position(i, servoState[i].goalPosition, nugus.servo_direction, nugus.servo_offset);
                }

                opencr.write(dynamixel::v2::SyncWriteCommand<DynamixelServoWriteDataPart1, 20>(
                    uint16_t(DynamixelIndirect::SERVO_WRITE_LOCATION_1),
                    data1));
                opencr.write(dynamixel::v2::SyncWriteCommand<DynamixelServoWriteDataPart2, 20>(
                    uint16_t(DynamixelIndirect::SERVO_WRITE_LOCATION_2),
                    data2));
            }

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

            // Get updated servo data
            // SYNC_READ (read the same memory addresses on all devices)
            for (int i = 0; i < 20; ++i) {
                packet_queue[i].push_back(PacketTypes::SERVO_DATA);
            }
            opencr.write(dynamixel::v2::SyncReadCommand<20>(uint16_t(DynamixelIndirect::SERVO_READ_LOCATION),
                                                            sizeof(DynamixelServoReadData),
                                                            nugus.servo_ids()));

            // Get OpenCR data
            // READ (only reading from a single device here)
            packet_queue[uint8_t(NUgus::ID::OPENCR)].push_back(PacketTypes::OPENCR_DATA);
            opencr.write(dynamixel::v2::ReadCommand(uint8_t(NUgus::ID::OPENCR),
                                                    uint16_t(OpenCR::Address::LED),
                                                    sizeof(OpenCRReadData)));


            // TODO: Find a way to gather received data and combine into a Sensors message for emitting
        });

        on<Trigger<ServoTargets>>().then([this](const ServoTargets& commands) {
            // Loop through each of our commands and update servo state information accordingly
            for (const auto& command : commands.targets) {
                float diff = utility::math::angle::difference(command.position, servoState[command.id].presentPosition);
                NUClear::clock::duration duration = command.time - NUClear::clock::now();

                float speed;
                if (duration.count() > 0) {
                    speed = diff / (double(duration.count()) / double(NUClear::clock::period::den));
                }
                else {
                    speed = 0;
                }

                // Update our internal state
                if (servoState[command.id].velocityPGain != command.gain
                    || servoState[command.id].velocityIGain != command.gain * 0
                    || servoState[command.id].velocityDGain != command.gain * 0
                    || servoState[command.id].goalVelocity != speed
                    || servoState[command.id].goalPosition != command.position) {

                    servoState[command.id].dirty = true;

                    servoState[command.id].velocityPGain = command.gain;
                    servoState[command.id].velocityIGain = command.gain * 0;
                    servoState[command.id].velocityDGain = command.gain * 0;

                    servoState[command.id].goalVelocity = speed;
                    servoState[command.id].goalPosition = command.position;
                }
            }
        });

        on<Trigger<ServoTarget>>().then([this](const ServoTarget& command) {
            auto commandList = std::make_unique<ServoTargets>();
            commandList->targets.push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::DIRECT>(std::move(commandList));
        });

        // If we get a HeadLED command then write it
        on<Trigger<RawSensors::HeadLED>>().then([this](const RawSensors::HeadLED& led) {
            // Update our internal state
            opencrState.headLED = led.RGB;
            opencrState.dirty   = true;
        });

        // If we get a EyeLED command then write it
        on<Trigger<RawSensors::EyeLED>>().then([this](const RawSensors::EyeLED& /*led*/) {
            // Update our internal state
            // OpenCR can only use 1 RGB LED
        });

        // If we get a EyeLED command then write it
        on<Trigger<RawSensors::LEDPanel>>().then([this](const RawSensors::LEDPanel& led) {
            // Update our internal state
            opencrState.ledPanel.led2 = led.led2;
            opencrState.ledPanel.led3 = led.led3;
            opencrState.ledPanel.led4 = led.led4;
            opencrState.dirty         = true;
        });

        on<Trigger<StatusReturn>>().then([this](const StatusReturn& packet) {
            // Figure out what the contents of the message are
            if (packet_queue[packet.id].size() > 0) {
                // Pop the front of the packet queue
                auto info = packet_queue[packet.id].front();
                packet_queue[packet.id].erase(packet_queue[packet.id].begin());

                switch (info) {
                    // Handles OpenCR model and version information
                    case PacketTypes::MODEL_INFORMATION: processModelInformation(packet); break;

                    // Handles OpenCR sensor data
                    case PacketTypes::OPENCR_DATA: processOpenCRData(packet); break;

                    // Handles servo data
                    case PacketTypes::SERVO_DATA: processServoData(packet); break;

                    // What is this??
                    default: log<NUClear::WARN>("Unknown packet data received"); break;
                }
            }

            else {
                log<NUClear::WARN>(fmt::format("Unexpected packet data received for ID {}.", packet.id));
            }
        });

        // When we receive data back from the OpenCR it will arrive here
        // Run a state machine to handle reception of packet header and data
        on<IO>(opencr.native_handle(), IO::READ).then([this] {
            enum class Phases : uint8_t { IDLE, HEADER_SYNC, PREAMBLE, DATA, FINISH, TIMEOUT };

            static constexpr uint8_t packet_header[4]           = {0xFF, 0xFF, 0xFD, 0x00};
            static Phases current_phase                         = Phases::IDLE;
            static uint8_t sync_point                           = 0;
            static NUClear::clock::time_point packet_start_time = NUClear::clock::now();
            static std::chrono::microseconds timeout            = std::chrono::microseconds(packet_wait);
            static std::vector<uint8_t> response;
            static uint16_t packet_length = 0;

            std::array<uint8_t, 128> buf;
            uint8_t num_bytes;

            num_bytes = opencr.read(buf.data(), 128);

            // Quick lambda to emit a completed StatusReturn message (to reduce code duplication)
            auto emit_msg = [&]() -> void {
                std::unique_ptr<StatusReturn> msg;
                msg->magic       = 0x00FDFFFF;
                msg->id          = response[4];
                msg->length      = packet_length;
                msg->instruction = response[7];
                msg->alert       = (response[8] >> 7) & 1;   // select alert flag in MSB
                msg->error       = response[8] & ~(1 << 7);  // remove alert flag to get error number
                std::copy(response.begin() + 9, response.begin() + packet_length - 4, std::back_inserter(msg->data));
                msg->checksum  = (response[response.size() - 1] << 8) | response[response.size() - 2];
                msg->timestamp = NUClear::clock::now();

                // Check CRC
                if (dynamixel::v2::calculateChecksum(response) == msg->checksum) {
                    emit(msg);
                }
                else {
                    log<NUClear::WARN>("Invalid CRC detected.");
                }
            };

            // Quick lambda to reset state machine state (to reduce code duplication)
            auto reset_state = [&]() -> void {
                sync_point    = 0;
                packet_length = 0;
                current_phase = Phases::IDLE;
                response.clear();
            };

            for (uint8_t i = 0; i < num_bytes; ++i) {
                switch (current_phase) {
                    // Idle phase
                    // We haven't seen any data, so we are waiting for the
                    // first byte of the header
                    case Phases::IDLE:
                        // If we match the first byte of the header then
                        // transition to the HEADER_SYNC phase
                        if (packet_header[sync_point] == buf[i]) {
                            response.push_back(packet_header[sync_point]);
                            sync_point++;
                            current_phase     = Phases::HEADER_SYNC;
                            packet_start_time = NUClear::clock::now();
                            timeout           = std::chrono::microseconds(packet_wait);
                        }
                        break;

                    // Header Sync phase
                    // We have matched the first byte of the header
                    // now match the next three bytes
                    case Phases::HEADER_SYNC:
                        if (NUClear::clock::now() < (packet_start_time + timeout)) {
                            if (packet_header[sync_point] == buf[i]) {
                                response.push_back(packet_header[sync_point]);
                                sync_point++;
                            }

                            // Header has been matched
                            // Now read in the rest of the packet
                            if (sync_point == 4) {
                                sync_point        = 0;
                                current_phase     = Phases::PREAMBLE;
                                packet_start_time = NUClear::clock::now();
                                timeout           = std::chrono::microseconds(byte_wait * 5 + 2000 + packet_wait);
                            }
                        }
                        else {
                            current_phase = Phases::TIMEOUT;
                        }
                        break;

                    // Preamble phase
                    // We have the full header, now we are looking for the next 5 bytes
                    // Packet ID, Length, Instruction ID, and Error
                    case Phases::PREAMBLE:
                        if (NUClear::clock::now() < (packet_start_time + timeout)) {
                            response.push_back(buf[i]);

                            // We just read in the expected length of the packet
                            if (response.size() == 7) {
                                packet_length = (response[6] << 8) | response[5];
                            }

                            // We now have the header and the packet preamble
                            // Time to get the packet parameters and CRC
                            if (response.size() == 9) {
                                current_phase = Phases::DATA;
                                timeout = std::chrono::microseconds(byte_wait * packet_length + 2000 + packet_wait);
                            }
                        }
                        else {
                            current_phase = Phases::TIMEOUT;
                        }
                        break;

                    // Data phase
                    // Header and preamble have been received
                    // Now we read in the message parameters and CRC
                    // We should be looking for (packet_length - 2) bytes
                    case Phases::DATA:
                        if (NUClear::clock::now() < (packet_start_time + timeout)) {
                            response.push_back(buf[i]);

                            // We now have all of our data
                            if (response.size() == size_t(7 + packet_length)) {
                                current_phase = Phases::FINISH;
                            }
                        }
                        else {
                            current_phase = Phases::TIMEOUT;
                        }
                        break;

                    // Finish phase
                    // We have now received a complete message
                    // However, it looks like we have more data to process
                    // Package up the current message, and reset our buf counter and phase
                    case Phases::FINISH:
                        emit_msg();

                        // Set up for next message
                        i--;  // Decrement counter to account for the next increment
                        reset_state();
                        break;

                    // Timeout phase
                    // It took too long to read in the full packet .... Giving up
                    case Phases::TIMEOUT:
                        log<NUClear::WARN>("Packet timeout occurred.");
                        reset_state();
                        break;

                    // Yea, dont know what happened here
                    default: reset_state(); break;
                }
            }

            // Our input buffer ended at the exact end of the packet
            if (response.size() == size_t(7 + packet_length)) {
                emit_msg();
                reset_state();
            }
        });
    }

    void HardwareIO::processModelInformation(const StatusReturn& packet) {
        uint16_t model  = (packet.data[1] << 8) | packet.data[0];
        uint8_t version = packet.data[2];
        log<NUClear::INFO>(fmt::format("OpenCR Model...........: {:#06X}", model));
        log<NUClear::INFO>(fmt::format("OpenCR Firmware Version: {:#04X}", version));
    }

    void HardwareIO::processOpenCRData(const StatusReturn& packet) {
        const OpenCRReadData data = *(reinterpret_cast<const OpenCRReadData*>(packet.data.data()));

        // 00000321
        // LED_1 = 0x01
        // LED_2 = 0x02
        // LED_3 = 0x04
        opencrState.ledPanel = {bool(data.led & 0x01), bool(data.led & 0x02), bool(data.led & 0x04)};

        // 0BBBBBGG GGGRRRRR
        // R = 0x001F
        // G = 0x02E0
        // B = 0x7C00
        uint32_t RGB = 0;
        RGB |= uint8_t(data.rgbLed & 0x001F) << 16;  // R
        RGB |= uint8_t(data.rgbLed & 0x02E0) << 8;   // G
        RGB |= uint8_t(data.rgbLed & 0x7C00);        // B
        opencrState.headLED = {RGB};

        // 00004321
        // Button_4 = Not used
        // Button Right (Reset) = 0x01
        // Button Middle = 0x02
        // Button Left = 0x04
        opencrState.buttons = {bool(data.button & 0x04), bool(data.button & 0x02), bool(data.button & 0x01)};

        opencrState.gyro = Eigen::Vector3f(convert::gyro(data.gyro[2]),   // X
                                           convert::gyro(data.gyro[1]),   // Y
                                           convert::gyro(data.gyro[0]));  // Z

        opencrState.acc = Eigen::Vector3f(convert::acc(data.acc[0]),   // X
                                          convert::acc(data.acc[1]),   // Y
                                          convert::acc(data.acc[2]));  // Z

        // Command send/receive errors only
        opencrState.alertFlag   = bool(packet.alert);
        opencrState.errorNumber = uint8_t(packet.error);

        // Work out a battery charged percentage
        batteryState.currentVoltage = convert::voltage(data.voltage);
        float percentage            = std::max(0.0f,
                                    (batteryState.currentVoltage - batteryState.flatVoltage)
                                        / (batteryState.chargedVoltage - batteryState.flatVoltage));

        // Battery percentage has changed, recalculate LEDs
        if (!utility::math::almost_equal(percentage, batteryState.percentage, 2)) {
            batteryState.dirty      = true;
            batteryState.percentage = percentage;

            uint32_t ledr            = 0;
            std::array<bool, 3> ledp = {false, false, false};

            if (batteryState.percentage > 0.90f) {
                ledp = {true, true, true};
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
            }
            else if (batteryState.percentage > 0.70f) {
                ledp = {false, true, true};
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
            }
            else if (batteryState.percentage > 0.50f) {
                ledp = {false, false, true};
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
            }
            else if (batteryState.percentage > 0.30f) {
                ledp = {false, false, false};
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
            }
            else if (batteryState.percentage > 0.20f) {
                ledp = {false, false, false};
                ledr = (uint8_t(0xFF) << 16) | (uint8_t(0x00) << 8) | uint8_t(0x00);
            }
            else if (batteryState.percentage > 0) {
                ledp = {false, false, false};
                ledr = (uint8_t(0xFF) << 16) | (uint8_t(0x00) << 8) | uint8_t(0x00);
            }
            // Error in reading voltage blue
            else {
                ledp = {false, false, false};
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0x00) << 8) | uint8_t(0xFF);
            }
            emit(std::make_unique<RawSensors::LEDPanel>(ledp[2], ledp[1], ledp[0]));
            emit(std::make_unique<RawSensors::HeadLED>(ledr));
        }
    }

    void HardwareIO::processServoData(const StatusReturn& packet) {
        const DynamixelServoReadData data = *(reinterpret_cast<const DynamixelServoReadData*>(packet.data.data()));

        servoState[packet.id].torqueEnabled   = (data.torqueEnable == 1);
        servoState[packet.id].errorFlags      = data.hardwareErrorStatus;
        servoState[packet.id].presentPWM      = convert::PWM(data.presentPWM);
        servoState[packet.id].presentCurrent  = convert::current(data.presentCurrent);
        servoState[packet.id].presentVelocity = convert::velocity(data.presentVelocity);
        servoState[packet.id].presentPosition =
            convert::position(packet.id, data.presentPosition, nugus.servo_direction, nugus.servo_offset);
        servoState[packet.id].voltage     = convert::voltage(data.presentVoltage);
        servoState[packet.id].temperature = convert::temperature(data.presentTemperature);
    }

    /**
     * @brief Creates a RawSensors message based on the current recorded states from the other processes.
     */
    RawSensors HardwareIO::constructSensors() {
        RawSensors sensors;

        // Timestamp when this message was created (data itsself could be old)
        sensors.time_stamp = NUClear::clock::now();

        /* OpenCR data */
        sensors.platform_error_flags = RawSensors::Error::OK;
        /// @todo Add proper error handling to translate new errors into rawsensors errors, using
        /// opencrState.errorFlags.errorNumber and opencrState.errorFlags.alertFlag
        sensors.led_panel     = opencrState.ledPanel;
        sensors.head_led      = opencrState.headLED;
        sensors.eye_led       = opencrState.eyeLED;
        sensors.buttons       = opencrState.buttons;
        sensors.accelerometer = opencrState.acc;
        sensors.gyroscope     = opencrState.gyro;

        /* Battery data */
        sensors.battery = batteryState.currentVoltage;

        /* Servos data */
        /// @todo unfuck the names of the servo fields. all fucked up between versions
        /// or just even wrong straight up
        for (int i = 0; i < 20; i++) {
            // Get a reference to the servo we are populating
            RawSensors::Servo& servo = utility::platform::getRawServo(i, sensors);


            // Booleans
            servo.torque_enabled = servoState[i].torqueEnabled;

            // Gain
            // RawSensors only takes PID but the v2 protocol has more options
            // so we assing the velocity gain to the sensor message
            servo.velocity_p_gain = servoState[i].velocityPGain;
            servo.velocity_i_gain = servoState[i].velocityIGain;
            servo.velocity_d_gain = servoState[i].velocityDGain;

            // Targets
            servo.goal_position    = servoState[i].goalPosition;
            servo.profile_velocity = servoState[i].presentVelocity;


            // If we are faking this hardware, simulate its motion
            if (servoState[i].simulated) {
                // Work out how fast we should be moving
                // 5.236 == 50 rpm which is similar to the max speed of the servos
                float movingSpeed =
                    (servoState[i].presentVelocity == 0 ? 5.236 : servoState[i].presentVelocity) / UPDATE_FREQUENCY;

                // Get our offset for this servo and apply it
                // The values are now between -pi and pi around the servos axis
                auto offset  = nugus.servo_offset[i];
                auto present = utility::math::angle::normalizeAngle(servoState[i].presentPosition - offset);
                auto goal    = utility::math::angle::normalizeAngle(servoState[i].goalPosition - offset);

                // We have reached our destination
                if (std::abs(present - goal) < movingSpeed) {
                    servoState[i].presentPosition = servoState[i].goalPosition;
                    servoState[i].presentVelocity = 0;
                }
                // We have to move towards our destination at moving speed
                else {
                    servoState[i].presentPosition = utility::math::angle::normalizeAngle(
                        (present + movingSpeed * (goal > present ? 1 : -1)) + offset);
                    servoState[i].presentVelocity = movingSpeed;
                }

                // Store our simulated values
                servo.present_position = servoState[i].presentPosition;
                servo.goal_position    = servoState[i].goalPosition;
                // servo.load             = 0;  // doesn't exist in v2 protocol
                servo.voltage     = servoState[i].voltage;
                servo.temperature = servoState[i].temperature;
            }

            // If we are using real data, get it from the packet
            else {
                // Error code
                servo.error_flags = servoState[i].errorFlags;

                // Present Data
                servo.present_position =
                    convert::position(i, servoState[i].presentPosition, nugus.servo_direction, nugus.servo_offset);
                servo.present_speed = convert::velocity(i, servoState[i].presentVelocity);

                // Diagnostic Information
                servo.voltage     = convert::voltage(servoState[i].voltage);
                servo.temperature = convert::temperature(servoState[i].temperature);

                // Clear Overvoltage flag if current voltage is greater than maximum expected voltage
                if (servo.voltage <= batteryState.chargedVoltage) {
                    servo.error_flags &= ~RawSensors::Error::INPUT_VOLTAGE;
                }
            }
        }

        /* FSRs data */

        return sensors;
    }

}  // namespace module::platform::openCR
