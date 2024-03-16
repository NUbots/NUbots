#include "HardwareIO.hpp"

#include <cmath>
#include <fmt/format.h>
#include <sstream>

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"
#include "message/platform/NUSenseData.hpp"

namespace module::platform::NUsense {

    using extension::Configuration;
    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;
    using message::platform::NUSense;
    using message::platform::RawSensors;

    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), nusense() {

        on<Configuration>("HardwareIO.yaml").then([this](const Configuration& config) {
            // Use configuration here from file HardwareIO.yaml
            this->log_level  = config["log_level"].as<NUClear::LogLevel>();
            cfg.nusense.port = config["nusense"]["port"].as<std::string>();
            cfg.nusense.baud = config["nusense"]["baud"].as<unsigned int>();

            nusense = utility::io::uart(cfg.nusense.port, cfg.nusense.baud);

            log<NUClear::INFO>("PORT OPENED");
        });

        on<Shutdown>().then("NUSense HardwareIO Shutdown", [this] {
            // Close our connection to the OpenCR
            if (nusense.connected()) {
                nusense.close();
            }
        });

        // When we receive data back from NUsense, we handle it here
        on<IO>(nusense.native_handle(), IO::READ).then([this] {
            // Read from NUsense
            uint32_t num_bytes = nusense.read(nusense_usb_bytes.data(), 512);

            nusense_receiver.receive(num_bytes, nusense_usb_bytes.data());

            // If packet successfully decoded
            if (nusense_receiver.handle()) {
                const auto& nusense_msg = nusense_receiver.get_nusense_message();

                log<NUClear::DEBUG>(
                    fmt::format("\nIMU Data\n"
                                "\tAccel(xyz): {} - {} - {}\n"
                                "\t Gyro(xyz): {} - {} - {}\n ",
                                nusense_msg.imu.accel.x,
                                nusense_msg.imu.accel.y,
                                nusense_msg.imu.accel.z,
                                nusense_msg.imu.gyro.x,
                                nusense_msg.imu.gyro.y,
                                nusense_msg.imu.gyro.z));

                log<NUClear::DEBUG>("Logging servo states...");

                for (const auto& [key, val] : nusense_msg.servo_map) {
                    log<NUClear::DEBUG>(fmt::format("      key: {}", key));

                    log<NUClear::DEBUG>(fmt::format("       id: {}", val.id));
                    log<NUClear::DEBUG>(fmt::format("   hw_err: {}", val.hardware_error));
                    log<NUClear::DEBUG>(fmt::format("torque_en: {}", val.torque_enabled));
                    log<NUClear::DEBUG>(fmt::format("     ppwm: {}", val.present_pwm));
                    log<NUClear::DEBUG>(fmt::format("    pcurr: {}", val.present_current));
                    log<NUClear::DEBUG>(fmt::format("    pvelo: {}", val.present_velocity));
                    log<NUClear::DEBUG>(fmt::format("     ppos: {}", val.present_position));
                    log<NUClear::DEBUG>(fmt::format("     gpwm: {}", val.goal_pwm));
                    log<NUClear::DEBUG>(fmt::format("    gcurr: {}", val.goal_current));
                    log<NUClear::DEBUG>(fmt::format("    gvelo: {}", val.goal_velocity));
                    log<NUClear::DEBUG>(fmt::format("     gpos: {}", val.goal_position));
                    log<NUClear::DEBUG>(fmt::format("  voltage: {}", val.voltage));
                    log<NUClear::DEBUG>(fmt::format("     temp: {}", val.temperature));
                }

                // emit the nusense msg to be captured by the reactino below.
                emit<Scope::DIRECT>(std::move(nusense_msg));
            }
        });

        // handle successfully decoded NUsense data
        on<Trigger<NUSense>>().then([this](const NUSense& data) {
            /* temp message to fill up */
            RawSensors sensors;

            /* Timestamp when this message was created because the incoming packet doesn't have a timestamp*/
            sensors.timestamp = NUClear::clock::now();

            /* Subcontroller data */
            sensors.subcontroller_error = 0;  // not yet implemented
            sensors.led_panel           = 0;  // not yet implemented
            sensors.head_led            = 0;  // not yet implemented
            sensors.eye_led             = 0;  // not yet implemented
            sensors.buttons             = 0;  // not yet implemented

            /* IMU */
            sensors.accelerometer = data.imu.accel;
            sensors.gyroscope     = data.imu.gyro;

            /* Battery data */
            sensors.battery = 0;  // not yet implemented

            /* Servo data */
            for (const auto& [key, val] : data.servo_map) {
                // Get a reference to the servo we are populating
                RawSensors::Servo& servo = utility::platform::getRawServo(val.id - 1, sensors);
                // fill all servo values from the reference
                servo.hardware_error        = val.hardware_error;
                servo.torque_enabled        = val.torque_enabled;
                servo.velocity_i_gain       = 0;  // not present in NUsense message
                servo.velocity_p_gain       = 0;  // not present in NUsense message
                servo.position_d_gain       = 0;  // not present in NUsense message
                servo.position_i_gain       = 0;  // not present in NUsense message
                servo.position_p_gain       = 0;  // not present in NUsense message
                servo.feed_forward_1st_Gain = 0;  // not present in NUsense message
                servo.feed_forward_2nd_Gain = 0;  // not present in NUsense message
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
                servo.profile_acceleration  = 0;  // not present in NUsense message
                servo.profile_velocity      = 0;  // not present in NUsense message
            }

            /* release to SensorFilter */
            emit(std::make_unique<RawSensors>(construct_sensors()));
        });


        on<Trigger<ServoTargets>>().then([this](const ServoTargets& commands) {
            // Write the command as one vector. ServoTargets messages are usually greater than 512 bytes but less
            // than 1024. This means that the USB2.0 protocol will split this up and will be received on the nusense
            // side as chunks of 512 as 512 bytes is the maximum bulk size that 2.0 allows. This also implies that the
            // read callback in the nusense side will be triggered at least twice (avg bytes for 20 filled servo targets
            // is about 700).
            std::array<char, 3> header = {(char) 0xE2, (char) 0x98, (char) 0xA2};

            std::vector<uint8_t> payload = NUClear::util::serialise::Serialise<ServoTargets>::serialise(commands);

            int payload_length                  = payload.size();
            uint8_t high_byte                   = (payload_length >> 8) & 0xFF;
            uint8_t low_byte                    = payload_length & 0xFF;
            std::array<uint8_t, 2> byte_lengths = {high_byte, low_byte};

            std::vector<char> full_msg;
            full_msg.insert(full_msg.end(), header.begin(), header.end());
            full_msg.insert(full_msg.end(), byte_lengths.begin(), byte_lengths.end());
            full_msg.insert(full_msg.end(), payload.begin(), payload.end());

            nusense.write(full_msg.data(), full_msg.size());

            // Logging
            log<NUClear::DEBUG>("Servo targets received");

            uint16_t total_length = static_cast<uint16_t>(high_byte << 8) | static_cast<uint16_t>(low_byte);

            log<NUClear::DEBUG>(total_length);
            log<NUClear::DEBUG>(fmt::format("header length: {}  length length: {}  payload length: {}",
                                            header.size(),
                                            byte_lengths.size(),
                                            payload.size()));
        });

        on<Trigger<ServoTarget>>().then([this](const ServoTarget& command) {
            auto command_list = std::make_unique<ServoTargets>();
            command_list->targets.push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::DIRECT>(std::move(command_list));

            log<NUClear::DEBUG>("Servo target received");
        });
    }

}  // namespace module::platform::NUsense
