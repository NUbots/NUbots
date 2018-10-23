#ifndef MODULE_PLATFORM_NUGUS_HARDWAREIO_H
#define MODULE_PLATFORM_NUGUS_HARDWAREIO_H

#include <nuclear>

#include "utility/io/uart.h"

namespace module {
namespace platform {
    namespace nugus {

        class HardwareIO : public NUClear::Reactor {

        public:
            /// @brief Called by the powerplant to build and setup the HardwareIO reactor.
            explicit HardwareIO(std::unique_ptr<NUClear::Environment> environment);

        private:
            utility::io::uart opencr;

            uint32_t byte_wait;
            uint32_t packet_wait;
        };

    }  // namespace nugus
}  // namespace platform
}  // namespace module

#endif  // MODULE_PLATFORM_NUGUS_HARDWAREIO_H
