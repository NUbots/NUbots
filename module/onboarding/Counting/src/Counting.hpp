#ifndef MODULE_ONBOARDING_COUNTING_HPP
#define MODULE_ONBOARDING_COUNTING_HPP

#include <nuclear>

namespace module::onboarding {

class Counting : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Counting reactor.
    explicit Counting(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::onboarding

#endif  // MODULE_ONBOARDING_COUNTING_HPP
