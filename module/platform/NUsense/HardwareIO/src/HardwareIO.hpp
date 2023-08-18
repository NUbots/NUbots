#ifndef MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP
#define MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP

#include <nuclear>
#include <vector>
#include "utility/io/uart.hpp"

namespace module::platform::NUsense {

    class HardwareIO : public NUClear::Reactor {

    public:
    /// @brief Called by the powerplant to build and setup the HardwareIO reactor.
    explicit HardwareIO(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief  Configuration variables for this reactor
        struct Config {
            std::string port = "";
        } cfg;

        /// @brief Manage desired port for NUSense
        utility::io::uart nusense{};
    };

}  // namespace module::platform::NUsense

#endif  // MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP
