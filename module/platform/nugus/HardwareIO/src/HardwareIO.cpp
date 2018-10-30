#include "HardwareIO.h"

#include <fmt/format.h>

#include "extension/Configuration.h"

#include "dynamixel/v2/Dynamixel.hpp"

#include "message/motion/ServoTarget.h"

#include "utility/math/angle.h"
#include "utility/support/yaml_expression.h"

namespace module {
namespace platform {
    namespace nugus {

        using extension::Configuration;

        using message::motion::ServoTarget;
        using message::platform::nugus::Sensors;
        using message::platform::nugus::StatusReturn;

        using utility::support::Expression;

        HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), opencr(), nugus(), byte_wait(0), packet_wait(0), packet_queue() {

            on<Configuration>("HardwareIO.yaml").then([this](const Configuration& config) {
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
                opencr.write(dynamixel::v2::WriteCommand<uint8_t>(
                    uint8_t(NUgus::ID::OPENCR), uint16_t(OpenCR::Address::STATUS_RETURN_LEVEL), uint8_t(1)));

                // Set the OpenCRs delay time to 0 (it may not have been configured before)
                opencr.write(dynamixel::v2::WriteCommand<uint8_t>(
                    uint8_t(NUgus::ID::OPENCR), uint16_t(OpenCR::Address::RETURN_DELAY_TIME), uint8_t(0)));

                // Find OpenCR firmware and model versions
                packet_queue[uint8_t(NUgus::ID::OPENCR)].push_back(PacketTypes::MODEL_INFORMATION);
                opencr.write(dynamixel::v2::ReadCommand(
                    uint8_t(NUgus::ID::OPENCR), uint16_t(OpenCR::Address::MODEL_NUMBER_L), uint8_t(3)));

                // Enable power to the servos
                opencr.write(dynamixel::v2::WriteCommand<uint8_t>(
                    uint8_t(NUgus::ID::OPENCR), uint16_t(OpenCR::Address::DYNAMIXEL_POWER), uint8_t(1)));

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
                    uint16_t(MX64::Address::INDIRECT_ADDRESS_1_L), read_data));

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
                    uint16_t(MX64::Address::INDIRECT_ADDRESS_18_L), write_data1));
                opencr.write(dynamixel::v2::SyncWriteCommand<std::array<uint16_t, 24>, 20>(
                    uint16_t(MX64::Address::INDIRECT_ADDRESS_29_L), write_data2));
            });

            on<Shutdown>().then("HardwareIO Startup", [this] {
                // Close our connection to the OpenCR
                if (opencr.connected()) {
                    opencr.close();
                }
            });

            // This trigger gets the sensor data from the CM730
            on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Single, Priority::HIGH>().then(
                "Hardware Loop", [this] {
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
                            data1[i].data.velocityPGain = nugus.convertPGain(servoState[i].velocityPGain);
                            data1[i].data.velocityIGain = nugus.convertIGain(servoState[i].velocityIGain);
                            data1[i].data.velocityDGain = nugus.convertDGain(servoState[i].velocityDGain);
                            data1[i].data.positionPGain = nugus.convertPGain(servoState[i].positionPGain);
                            data1[i].data.positionIGain = nugus.convertIGain(servoState[i].positionIGain);

                            data2[i].data.feedforward1stGain  = nugus.convertFFGain(servoState[i].feedforward1stGain);
                            data2[i].data.feedforward2ndGain  = nugus.convertFFGain(servoState[i].feedforward2ndGain);
                            data2[i].data.goalPWM             = nugus.convertPWM(servoState[i].goalPWM);
                            data2[i].data.goalCurrent         = nugus.convertCurrent(servoState[i].goalCurrent);
                            data2[i].data.goalVelocity        = nugus.convertVelocity(servoState[i].goalVelocity);
                            data2[i].data.profileAcceleration = nugus.convertFFGain(servoState[i].profileAcceleration);
                            data2[i].data.profileVelocity     = nugus.convertFFGain(servoState[i].profileVelocity);
                            data2[i].data.goalPosition        = nugus.convertPosition(i, servoState[i].goalPosition);
                        }

                        opencr.write(dynamixel::v2::SyncWriteCommand<DynamixelServoWriteDataPart1, 20>(
                            uint16_t(MX64::Address::INDIRECT_DATA_18), data1));
                        opencr.write(dynamixel::v2::SyncWriteCommand<DynamixelServoWriteDataPart2, 20>(
                            uint16_t(MX64::Address::INDIRECT_DATA_29), data2));
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
                        opencr.write(dynamixel::v2::WriteCommand<OpenCRWriteData>(
                            uint8_t(NUgus::ID::OPENCR), uint16_t(OpenCR::Address::LED), data));
                    }

                    // Get updated servo data
                    // SYNC_READ (read the same memory addresses on all devices)
                    for (int i = 0; i < 20; ++i) {
                        packet_queue[i].push_back(PacketTypes::SERVO_DATA);
                    }
                    opencr.write(dynamixel::v2::SyncReadCommand<20>(
                        uint16_t(MX64::Address::INDIRECT_DATA_1), sizeof(DynamixelServoReadData), nugus.servo_ids()));

                    // Get OpenCR data
                    // READ (only reading from a single device here)
                    packet_queue[uint8_t(NUgus::ID::OPENCR)].push_back(PacketTypes::OPENCR_DATA);
                    opencr.write(dynamixel::v2::ReadCommand(
                        uint8_t(NUgus::ID::OPENCR), uint16_t(OpenCR::Address::LED), sizeof(OpenCRReadData)));


                    // TODO: Find a way to gather received data and combine into a Sensors message for emitting
                });

            // This trigger writes the servo positions to the hardware
            on<Trigger<std::vector<ServoTarget>>>().then([this](const std::vector<ServoTarget>& commands) {
                // Loop through each of our commands and update servo state information accordingly
                for (const auto& command : commands) {
                    float diff =
                        utility::math::angle::difference(command.position, servoState[command.id].presentPosition);
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
                auto commandList = std::make_unique<std::vector<ServoTarget>>();
                commandList->push_back(command);

                // Emit it so it's captured by the reaction above
                emit<Scope::DIRECT>(std::move(commandList));
            });

            // If we get a HeadLED command then write it
            on<Trigger<Sensors::HeadLED>>().then([this](const Sensors::HeadLED& led) {
                // Update our internal state
                opencrState.headLED = led.RGB;
                opencrState.dirty   = true;
            });

            // If we get a EyeLED command then write it
            on<Trigger<Sensors::EyeLED>>().then([this](const Sensors::EyeLED& /*led*/) {
                // Update our internal state
                // OpenCR can only use 1 RGB LED
            });

            // If we get a EyeLED command then write it
            on<Trigger<Sensors::LEDPanel>>().then([this](const Sensors::LEDPanel& led) {
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
                    msg->error       = static_cast<StatusReturn::CommandError>(response[8]);
                    std::copy(
                        response.begin() + 9, response.begin() + packet_length - 4, std::back_inserter(msg->data));
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
            opencrState.ledPanel = {data.led & 0x01, data.led & 0x02, data.led & 0x04};

            // 0BBBBBGG GGGRRRRR
            // R = 0x001F
            // G = 0x02E0
            // B = 0x7C00
            uint32_t RGB = uint8_t(data.rgbLed & 0x001F) << 16;
            RGB |= uint8_t(data.rgbLed & 0x02E0) << 8;
            RGB |= uint8_t(data.rgbLed & 0x7C00);
            opencrState.headLED = {RGB};
            opencrState.headLED = {RGB};

            // Frequency of the buzzer
            opencrState.buzzer = data.buzzer;

            // 00004321
            // Button_4 = Not used
            // Button Right (Reset) = 0x01
            // Button Middle = 0x02
            // Button Left = 0x04
            opencrState.buttons = {data.button & 0x04, data.button & 0x02, data.button & 0x01};

            opencrState.gyro = {nugus.convertGyro(data.gyro[2]),   // X
                                nugus.convertGyro(data.gyro[1]),   // Y
                                nugus.convertGyro(data.gyro[0])};  // Z

            opencrState.acc = {nugus.convertAcc(data.acc[0]),   // X
                               nugus.convertAcc(data.acc[1]),   // Y
                               nugus.convertAcc(data.acc[2])};  // Z

            opencrState.errorFlags = int(packet.error);

            // Work out a battery charged percentage
            batteryState.currentVoltage = nugus.convertVoltage(data.voltage);
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
                emit(std::make_unique<Sensors::LEDPanel>(ledp[2], ledp[1], ledp[0]));
                emit(std::make_unique<Sensors::HeadLED>(ledr));
            }
        }

        void HardwareIO::processServoData(const StatusReturn& packet) {
            const DynamixelServoReadData data = *(reinterpret_cast<const DynamixelServoReadData*>(packet.data.data()));

            servoState[packet.id].torqueEnabled   = (data.torqueEnable == 1);
            servoState[packet.id].errorFlags      = data.hardwareErrorStatus;
            servoState[packet.id].presentPWM      = nugus.convertPWM(data.presentPWM);
            servoState[packet.id].presentCurrent  = nugus.convertCurrent(data.presentCurrent);
            servoState[packet.id].presentVelocity = nugus.convertVelocity(data.presentVelocity);
            servoState[packet.id].presentPosition = nugus.convertPosition(packet.id, data.presentPosition);
            servoState[packet.id].voltage         = nugus.convertVoltage(data.presentVoltage);
            servoState[packet.id].temperature     = nugus.convertTemperature(data.presentTemperature);
        }
    }  // namespace nugus
}  // namespace platform
}  // namespace module
