#ifndef MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP
#define MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP

#include <nuclear>
#include <vector>

#include "util/packet_handler.hpp"

#include "utility/io/uart.hpp"

namespace module::platform::NUsense {

    class HardwareIO : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the HardwareIO reactor.
        explicit HardwareIO(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief  Configuration variables for this reactor
        struct Config {
            struct {
                std::string port = "";
                int baud         = 0;
            } nusense;
        } cfg;

        std::array<uint8_t, 512> nusense_usb_bytes{};

        /// @brief Manage desired port for NUSense
        utility::io::uart nusense;
        PacketHandler nusense_receiver;
    };

}  // namespace module::platform::NUsense

#endif  // MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP
