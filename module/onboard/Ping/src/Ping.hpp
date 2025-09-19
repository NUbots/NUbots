#ifndef MODULE_ONBOARD_PING_HPP
#define MODULE_ONBOARD_PING_HPP

#include <nuclear>

namespace module::onboard {

class Ping : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Ping reactor.
    explicit Ping(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::onboard

#endif  // MODULE_ONBOARD_PING_HPP
