#ifndef MODULE_ONBOARDING_EXPLOSION_HPP
#define MODULE_ONBOARDING_EXPLOSION_HPP

#include <nuclear>

namespace module::onboarding {

class Explosion : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Explosion reactor.
    explicit Explosion(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::onboarding

#endif  // MODULE_ONBOARDING_EXPLOSION_HPP
