#ifndef MODULE_ONBOARDING_STARTER_HPP
#define MODULE_ONBOARDING_STARTER_HPP

#include <nuclear>

namespace module::onboarding {

class Starter : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Starter reactor.
    explicit Starter(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::onboarding

#endif  // MODULE_ONBOARDING_STARTER_HPP
