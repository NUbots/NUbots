#include "FirmwareInstaller.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>

#include "extension/Configuration.h"

#include "utility/file/fileutil.h"
#include "utility/io/uart.h"
#include "utility/strutil/strutil.h"

namespace module {
namespace tools {

    using extension::Configuration;

    struct FlashCM730 {};
    struct FlashComplete {};

    FirmwareInstaller::FirmwareInstaller(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , device("UNKNOWN")
        , menu_state(DEVICE_MENU)
        , selected_device(NO_DEVICE)
        , selected_battery(NO_BATTERY) {

        on<Configuration>("FirmwareInstaller.yaml").then([this](const Configuration& config) {
            device = config["device"].as<std::string>();

            for (auto& f : config["firmwares"].config) {
                std::pair<std::string, std::string> name;
                name.first       = utility::strutil::toUpper(f["device"].as<std::string>());
                name.second      = utility::strutil::toUpper(f["battery"].as<std::string>());
                std::string path = f["path"].as<std::string>();

                Firmware fw;

                if (utility::file::exists(path)) {
                    auto ifs = std::ifstream(path, std::ios::in | std::ios::binary);

                    fw.firmware =
                        std::vector<uint8_t>((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
                    fw.checksum = std::accumulate(fw.firmware.begin(), fw.firmware.end(), uint8_t(0));
                    firmwares.insert(std::make_pair(name, fw));
                }
                else {
                    log<NUClear::WARN>("The firmware file", path, "for", name.first, "does not exist");
                }
            }
        });

        showDeviceMenu();

        on<IO>(STDIN_FILENO, IO::READ).then([this] {
            // Get input.
            std::string input;
            std::cin >> input;

            // Convert input to uppercase.
            input = utility::strutil::toUpper(input);

            switch (menu_state) {
                // Ignore inputs.
                case NO_MENU: break;

                case DEVICE_MENU:
                    if (input.compare("CM730") == 0) {
                        selected_device  = CM730;
                        selected_battery = NO_BATTERY;
                        menu_state       = BATTERY_MENU;
                        showBatteryMenu();
                    }

                    else if (input.compare("DYNXL") == 0) {
                        selected_device  = DYNAMIXEL;
                        selected_battery = NO_BATTERY;
                        menu_state       = DEVICE_MENU;  // This will change when we implement this feature.
                        std::cout << "Too bad. We haven't implemented this yet." << std::endl;
                        showDeviceMenu();
                    }

                    else if (input.compare("QUIT") == 0) {
                        selected_device  = NO_DEVICE;
                        selected_battery = NO_BATTERY;
                        std::cout << "Bye." << std::endl;
                        powerplant.shutdown();
                    }

                    else {
                        selected_device  = NO_DEVICE;
                        selected_battery = NO_BATTERY;
                        std::cout << "Unknown input. Try again." << std::endl;
                        showDeviceMenu();
                    }

                    break;

                case BATTERY_MENU:
                    if (input.compare("1") == 0) {
                        selected_battery = BATTERY3;
                        menu_state       = NO_MENU;
                        emit(std::make_unique<FlashCM730>());
                    }

                    else if (input.compare("2") == 0) {
                        selected_battery = BATTERY4;
                        menu_state       = NO_MENU;
                        emit(std::make_unique<FlashCM730>());
                    }

                    else if (input.compare("0") == 0) {
                        selected_device  = NO_DEVICE;
                        selected_battery = NO_BATTERY;
                        menu_state       = DEVICE_MENU;
                        showDeviceMenu();
                    }

                    else {
                        selected_battery = NO_BATTERY;
                        std::cout << "Unknown input. Try again." << std::endl;
                        showBatteryMenu();
                    }

                    break;
            }
        });

        on<Trigger<FlashComplete>, Sync<FirmwareInstaller>>().then([this] {
            menu_state       = DEVICE_MENU;
            selected_device  = NO_DEVICE;
            selected_battery = NO_BATTERY;
            showDeviceMenu();
        });

        on<Trigger<FlashCM730>, Sync<FirmwareInstaller>>().then([this] {
            std::pair<std::string, std::string> name;

            switch (selected_device) {
                case CM730: name.first = "CM730"; break;
                default: log<NUClear::WARN>("Invalid device selected."); return;
            }

            switch (selected_battery) {
                case BATTERY3: name.second = "3CELL"; break;
                case BATTERY4: name.second = "4CELL"; break;
                default: log<NUClear::WARN>("Invalid battery selected."); return;
            }

            const auto& it = firmwares.find(name);

            // Open the UART
            utility::io::uart uart(device, 57600);

            if (!uart.connected()) {
                log<NUClear::WARN>("Unable to connect to CM730 firmware. Check your config file.");
                powerplant.shutdown();
            }

            else if (it == firmwares.end()) {
                log<NUClear::WARN>("Unable to find CM730 firmware. Check your config file.");
                powerplant.shutdown();
            }

            else {
                const auto& cm730 = it->second;

                pollfd pfd{uart.native_handle(), POLLIN, 0};

                log<NUClear::INFO>("Please press the CM730 reset button to begin...");

                // Wait for connection to the CM730 Bootloader
                char send      = '#';
                char recv[256] = {'\0'};
                int read       = 1;
                do {
                    std::cout << "\rWaiting for CM730 to reset .....";
                    uart.write(&send, sizeof(send));

                    // Wait for some action!
                    poll(&pfd, 1, 20);
                    read       = uart.read(&recv, sizeof(recv));
                    recv[read] = '\0';
                } while ((std::string(recv, read).compare("#") != 0) && powerplant.running());

                std::cout << "\rWaiting for CM730 to reset ....." << std::endl;

                // Check we are good to go
                if (std::string(recv, read).compare("#") == 0) {

                    log<NUClear::INFO>("CM730 reset complete...");

                    // Enter erase block mode and erase
                    uart.write("\rl\r", 3);

                    // Wait for the block to be erased
                    do {
                        poll(&pfd, 1, 135);
                        read = uart.read(recv, sizeof(recv));

                        // CM730 echos "l\r"
                        // std::string text(recv, read);
                        // log<NUClear::INFO>(text);
                    } while (read != 0);

                    log<NUClear::INFO>("Erase block complete...");

                    // Give the bootloader time to catch its breath
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));

                    // TODO write in such a way that you get progress
                    ssize_t count;
                    for (count = 0; static_cast<size_t>(count) < cm730.firmware.size();) {
                        ssize_t writeSize = 64;

                        if ((static_cast<size_t>(count) + 64) > cm730.firmware.size()) {
                            writeSize = cm730.firmware.size() - count;
                        }

                        if (uart.write(cm730.firmware.data() + count, writeSize) == writeSize) {
                            count += writeSize;
                            std::cout << "\rFlashing CM730 firmware " << count << "/" << cm730.firmware.size()
                                      << " completed...";
                        }
                    }

                    std::cout << "\rFlashing CM730 firmware " << count << "/" << cm730.firmware.size()
                              << " completed..." << std::endl;
                    std::cout << "Flashing CM730 firmware completed." << std::endl;

                    // Write the checksum
                    while (uart.write(&cm730.checksum, sizeof(cm730.checksum)) != sizeof(cm730.checksum)) {
                        ;
                    }

                    log<NUClear::INFO>("Downloading Bytesum...");

                    // Wait for the device to finish processing the firmware
                    while (read != 0) {
                        poll(&pfd, 1, 10);
                        read = uart.read(recv, sizeof(recv));

                        std::string text(recv, read);
                        log<NUClear::INFO>(text);
                    }

                    log<NUClear::INFO>("Firmware installation complete...");

                    std::this_thread::sleep_for(std::chrono::milliseconds(10));

                    // Exit the bootloader
                    uart.write("\rgo\r", 4);
                    poll(&pfd, 1, 50);
                    if ((read = uart.read(recv, sizeof(recv))) > 0) {
                        std::string text(recv, read);
                        log<NUClear::INFO>(text);
                    }

                    // Close our uart
                    uart.close();
                }

                else {
                    log<NUClear::WARN>("CM730 reset failed.");
                    uart.close();
                }
            }

            emit(std::make_unique<FlashComplete>());
        });
    }

    void FirmwareInstaller::showDeviceMenu() const {
        std::cout << "\n\n" << std::endl;
        std::cout << "***********************************" << std::endl;
        std::cout << "* Welcome to NUfirmware Installer *" << std::endl;
        std::cout << "***********************************" << std::endl;
        std::cout << "\tType \"CM730\" to flash CM730 firmware." << std::endl;
        std::cout << "\tType \"DYNXL\" to flash Dynamixel firmware." << std::endl;
        std::cout << "\tType \"QUIT\" to quit." << std::endl;
        std::cout << "Choice: " << std::flush;
    }

    void FirmwareInstaller::showBatteryMenu() const {
        std::cout << "\n" << std::endl;
        std::cout << "Select battery type: " << std::endl;
        std::cout << "\t1) 11.1V 3 Cell Battery" << std::endl;
        std::cout << "\t2) 14.8V 4 Cell Battery" << std::endl;
        std::cout << "\t0) Cancel" << std::endl;
        std::cout << "Choice: " << std::flush;
    }

}  // namespace tools
}  // namespace module
