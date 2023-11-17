#ifndef MODULE_ONBOARDING_CATALYST_HPP
#define MODULE_ONBOARDING_CATALYST_HPP

#include <nuclear>

namespace module::onboarding {

class Catalyst : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Catalyst reactor.
    explicit Catalyst(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::onboarding

#endif  // MODULE_ONBOARDING_CATALYST_HPP
