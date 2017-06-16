#include "FirmwareInstaller.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>

#include "extension/Configuration.h"

#include "utility/file/fileutil.h"
#include "utility/io/uart.h"

namespace module {
namespace tools {

using extension::Configuration;

struct FlashCM730 {};

FirmwareInstaller::FirmwareInstaller(
    std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)), device("UNKNOWN") {
  on<Configuration>("FirmwareInstaller.yaml")
      .then([this](const Configuration& config) {
        device = config["device"].as<std::string>();

        for (auto& f : config["firmware"].config) {
          std::string name = f.first.as<std::string>();
          std::string path = f.second.as<std::string>();

          Firmware fw;

          if (utility::file::exists(path)) {
            auto ifs = std::ifstream(path, std::ios::in | std::ios::binary);

            fw.firmware =
                std::vector<uint8_t>((std::istreambuf_iterator<char>(ifs)),
                                     std::istreambuf_iterator<char>());
            fw.checksum = std::accumulate(fw.firmware.begin(),
                                          fw.firmware.end(), uint8_t(0));

            firmwares.insert(std::make_pair(name, fw));
          } else {
            log<NUClear::WARN>("The firmware file", path, "for", name,
                               "does not exist");
          }
        }
      });

  on<IO>(STDIN_FILENO, IO::READ).then([this] {

    // TODO make this a robust and beautiful user interface
    std::string kgo;
    std::cin >> kgo;
    emit(std::make_unique<FlashCM730>());
  });

  on<Trigger<FlashCM730>, Sync<FirmwareInstaller>>().then([this] {

    const auto& it = firmwares.find("CM730");

    if (it == firmwares.end()) {
      log<NUClear::WARN>(
          "Unable to find CM730 firmware. Check your config file.");
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
  });
}

}  // namespace tools
}  // namespace module
