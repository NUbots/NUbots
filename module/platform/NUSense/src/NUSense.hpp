#ifndef MODULE_PLATFORM_NUSENSE_HPP
#define MODULE_PLATFORM_NUSENSE_HPP

#include <nuclear>

namespace module::platform {

class NUSense : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the NUSense reactor.
    explicit NUSense(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::platform

#endif  // MODULE_PLATFORM_NUSENSE_HPP
