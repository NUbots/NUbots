#ifndef MODULE_TOOLS_FIRMWAREINSTALLER_H
#define MODULE_TOOLS_FIRMWAREINSTALLER_H

#include <nuclear>

#include <string>

namespace module {
namespace tools {

class FirmwareInstaller : public NUClear::Reactor {
 public:
  /// @brief Called by the powerplant to build and setup the FirmwareInstaller
  /// reactor.
  explicit FirmwareInstaller(std::unique_ptr<NUClear::Environment> environment);

 private:
  std::string device;
  struct Firmware {
    std::vector<uint8_t> firmware;
    uint8_t checksum;
  };

  std::map<std::string, Firmware> firmwares;

  bool ignore_inputs;
};

}  // namespace tools
}  // namespace module

#endif  // MODULE_TOOLS_FIRMWAREINSTALLER_H
