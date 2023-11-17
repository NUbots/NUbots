#ifndef MODULE_ONBOARDING_REACTION_HPP
#define MODULE_ONBOARDING_REACTION_HPP

#include <nuclear>

namespace module::onboarding {

class Reaction : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Reaction reactor.
    explicit Reaction(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::onboarding

#endif  // MODULE_ONBOARDING_REACTION_HPP
