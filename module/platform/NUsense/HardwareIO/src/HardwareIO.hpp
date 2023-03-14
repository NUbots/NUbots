#ifndef MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP
#define MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP

#include <nuclear>

namespace module::platform::NUsense {

class HardwareIO : public NUClear::Reactor {
private:
    /// The configuration variables for this reactor
    struct {
    } config;

public:
    /// @brief Called by the powerplant to build and setup the HardwareIO reactor.
    explicit HardwareIO(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::platform::NUsense

#endif  // MODULE_PLATFORM_NUSENSE_HARDWAREIO_HPP
