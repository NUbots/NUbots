#ifndef MODULE_ONBOARDING_STARTCOUNTING_HPP
#define MODULE_ONBOARDING_STARTCOUNTING_HPP

#include <nuclear>

namespace module::onboarding {

class StartCounting : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the StartCounting reactor.
    explicit StartCounting(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::onboarding

#endif  // MODULE_ONBOARDING_STARTCOUNTING_HPP
