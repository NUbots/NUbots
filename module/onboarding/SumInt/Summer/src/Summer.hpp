#ifndef MODULE_ONBOARDING_SUMMER_HPP
#define MODULE_ONBOARDING_SUMMER_HPP

#include <nuclear>

namespace module::onboarding {

class Summer : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Summer reactor.
    explicit Summer(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::onboarding

#endif  // MODULE_ONBOARDING_SUMMER_HPP
