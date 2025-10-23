#ifndef MODULE_ONBOARDING_ACCUMULATOR_HPP
#define MODULE_ONBOARDING_ACCUMULATOR_HPP

#include <nuclear>

namespace module::onboarding {

class Accumulator : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Accumulator reactor.
    explicit Accumulator(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::onboarding

#endif  // MODULE_ONBOARDING_ACCUMULATOR_HPP
