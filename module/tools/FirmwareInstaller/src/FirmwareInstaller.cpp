#include "FirmwareInstaller.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>

#include "extension/Configuration.h"

#include "utility/file/fileutil.h"
#include "utility/io/uart.h"

namespace module {
namespace tools {

    using extension::Configuration;

    struct FlashCM730 {};

    FirmwareInstaller::FirmwareInstaller(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), device("UNKNOWN"), ignore_inputs(false) {
        on<Configuration>("FirmwareInstaller.yaml").then([this](const Configuration& config) {
            device = config["device"].as<std::string>();

            for (auto& f : config["firmware"].config) {
                std::string name = f.first.as<std::string>();
                std::string path = f.second.as<std::string>();

                Firmware fw;

                if (utility::file::exists(path)) {
                    auto ifs = std::ifstream(path, std::ios::in | std::ios::binary);

                    fw.firmware =
                        std::vector<uint8_t>((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
                    fw.checksum = std::accumulate(fw.firmware.begin(), fw.firmware.end(), uint8_t(0));

                    firmwares.insert(std::make_pair(name, fw));
                }
                else {
                    log<NUClear::WARN>("The firmware file", path, "for", name, "does not exist");
                }
            }
        });

        auto showMenu = [this]() -> void {
            std::cout << "\n\n" << std::endl;
            std::cout << "***********************************" << std::endl;
            std::cout << "* Welcome to NUfirmware Installer *" << std::endl;
            std::cout << "***********************************" << std::endl;
            std::cout << "\tType \"CM730\" to flash CM730 firmware." << std::endl;
            std::cout << "\tType \"DYNXL\" to flash Dynamixel firmware." << std::endl;
            std::cout << "\tType \"QUIT\" to quit." << std::endl;
        };

        showMenu();

        on<IO>(STDIN_FILENO, IO::READ).then([this, &showMenu] {

            if (!ignore_inputs) {
                std::string input;
                std::cin >> input;

                // Convert input to uppercase.
                std::transform(
                    input.begin(), input.end(), input.begin(), [](const char& c) -> char { return std::toupper(c); });

                if (input.compare("CM730") == 0) {
                    emit(std::make_unique<FlashCM730>());
                }

                else if (input.compare("DYNXL") == 0) {
                    std::cout << "Too bad. We haven't implemented this yet." << std::endl;
                }

                else if (input.compare("QUIT") == 0) {
                    std::cout << "Bye." << std::endl;
                    powerplant.shutdown();
                }

                else {
                    std::cout << "Unknown input. Try again." << std::endl;
                }

                showMenu();
            }
        });

        on<Trigger<FlashCM730>, Sync<FirmwareInstaller>>().then([this] {

            // Make sure the user can't quit part way through the process.
            ignore_inputs = true;

            const auto& it = firmwares.find("CM730");

            if (it == firmwares.end()) {
                log<NUClear::WARN>("Unable to find CM730 firmware. Check your config file.");
            }

            else {
                const auto& cm730 = it->second;

                // Open the UART
                utility::io::uart uart(device, 57600);

                pollfd pfd{uart.native_handle(), POLLIN, 0};

                log<NUClear::INFO>("Please press the CM730 reset button to begin...");

                // Wait for connection to the CM730 Bootloader
                char send = '#';
                char recv = '\0';
                while (recv != '#' && powerplant.running()) {
                    uart.write(&send, sizeof(send));

                    // Wait for some action!
                    poll(&pfd, 1, 10);
                    uart.read(&recv, sizeof(recv));
                }

                // Check we are good to go
                if (recv == '#') {
                    // Enter erase block mode and erase
                    uart.write("l\r", 2);

                    // Wait for the block to be erased
                    int read = 1;
                    char buff[256];
                    while (read != 0) {
                        poll(&pfd, 1, 100);
                        read = uart.read(buff, sizeof(buff));

                        std::string text(buff, read);
                        log<NUClear::INFO>(text);
                    }

                    log<NUClear::INFO>("Erase block complete...");

                    // Give the bootloader time to catch its breath
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));

                    // TODO write in such a way that you get progress
                    uart.write(cm730.firmware.data(), cm730.firmware.size());

                    // Write the checksum
                    uart.write(&cm730.checksum, sizeof(cm730.checksum));

                    // Wait for the device to finish processing the firmware
                    while (read != 0) {
                        poll(&pfd, 1, 10);
                        read = uart.read(buff, sizeof(buff));

                        std::string text(buff, read);
                        log<NUClear::INFO>(text);
                    }

                    // Exit the bootloader
                    uart.write("\rgo\r", 4);

                    // Close our uart
                    uart.close();
                }
            }

            ignore_inputs = false;
        });
    }

}  // namespace tools
}  // namespace module
