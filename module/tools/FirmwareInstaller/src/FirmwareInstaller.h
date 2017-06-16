#ifndef MODULE_TOOLS_FIRMWAREINSTALLER_H
#define MODULE_TOOLS_FIRMWAREINSTALLER_H

#include <nuclear>

#include <string>

namespace module {
namespace tools {

    class FirmwareInstaller : public NUClear::Reactor {

    public:
        static constexpr uint32_t MEMORY_MAXSIZE           = (256 * 1024); /* size in bytes */
        static constexpr uint32_t MAX_LINE_SIZE            = 1024;
        static constexpr uint32_t ADDRESS_MASK             = 0x000FFFF0;
        static constexpr uint32_t NO_ADDRESS_TYPE_SELECTED = 0;
        static constexpr uint32_t LINEAR_ADDRESS           = 1;
        static constexpr uint32_t SEGMENTED_ADDRESS        = 2;

        /// @brief Called by the powerplant to build and setup the FirmwareInstaller reactor.
        explicit FirmwareInstaller(std::unique_ptr<NUClear::Environment> environment);

    private:
        std::string device;
        struct Firmware {
            std::vector<uint8_t> firmware;
            uint8_t checksum;
        };

        std::map<std::string, Firmware> firmwares;
    };

}  // namespace tools
}  // namespace module

#endif  // MODULE_TOOLS_FIRMWAREINSTALLER_H
