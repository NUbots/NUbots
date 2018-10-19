#include "HardwareIO.h"

#include "extension/Configuration.h"

#include "OpenCR.h"
#include "dynamixel/v2/Dynamixel.hpp"

namespace module {
namespace platform {
    namespace nugus {

        using extension::Configuration;

        HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), opencr() {

            on<Configuration>("HardwareIO.yaml").then([this](const Configuration& config) {
                // Make sure OpenCR is operating at the correct baud rate (based on config params)
                if (opencr.connected()) {
                    opencr.close();
                }

                opencr.open(config["opencr"]["device"], config["opencr"]["baud"]);
            });

            on<Startup>().then("HardwareIO Startup", [this] {
                // Find OpenCR firmware and model versions
                dynamixel::v2::Read(OpenCR::ID, OpenCR::Address::MODEL_NUMBER_L, 3);

                // Enable power to the servos
                dynamixel::v2::Write(OpenCR::ID, OpenCR::Address::DYNAMIXEL_POWER, uint8_t(1));

                // Set up indirect addressing ... if needed
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
                    // Our final sensor output
                    auto sensors = std::make_unique<DarwinSensors>();

                    // Write out servo data
                    // SYNC_WRITE (write the same memory addresses on all devices)

                    // Get updated servo data
                    // SYNC_READ (read the same memory addresses on all devices)

                    // Get OpenCR data
                    // READ (only reading from a single device here)

                    // Parse our data
                    *sensors = parseSensors(data);

                    // Work out a battery charged percentage
                    sensors->battery =
                        std::max(0.0f, (sensors->voltage - flatVoltage) / (chargedVoltage - flatVoltage));

                    // Change LEDs to reflect battery voltage

                    // Send our nicely computed sensor data out to the world
                    emit(std::move(sensors));
                });

            // This trigger writes the servo positions to the hardware
            on<Trigger<std::vector<ServoTarget>>, With<DarwinSensors>>().then(
                [this](const std::vector<ServoTarget>& commands, const DarwinSensors& sensors) {
                    // Loop through each of our commands and update servo state information accordingly
                    for (const auto& command : commands) {
                    }
                });

            on<Trigger<ServoTarget>>().then([this](const ServoTarget command) {
                auto commandList = std::make_unique<std::vector<ServoTarget>>();
                commandList->push_back(command);

                // Emit it so it's captured by the reaction above
                emit<Scope::DIRECT>(std::move(commandList));
            });

            // If we get a HeadLED command then write it
            on<Trigger<DarwinSensors::HeadLED>>().then([this](const DarwinSensors::HeadLED& led) {
                // Update our internal state
            });

            // If we get a EyeLED command then write it
            on<Trigger<DarwinSensors::EyeLED>>().then([this](const DarwinSensors::EyeLED& led) {
                // Update our internal state
            });

            // If we get a EyeLED command then write it
            on<Trigger<DarwinSensors::LEDPanel>>().then([this](const DarwinSensors::LEDPanel& led) {
                // Update our internal state
            });
        }
    }  // namespace nugus
}  // namespace platform
}  // namespace module
