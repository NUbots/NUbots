#ifndef MODULE_ONBOARDING_PONG_HPP
#define MODULE_ONBOARDING_PONG_HPP

#include <nuclear>

namespace module::onboarding {

class Pong : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
        int nFinal = 10;
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Pong reactor.
    explicit Pong(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::onboarding

#endif  // MODULE_ONBOARDING_PONG_HPP
