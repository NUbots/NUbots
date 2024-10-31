#ifndef MODULE_ONBOARDING_DONECOUNTING_HPP
#define MODULE_ONBOARDING_DONECOUNTING_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::onboarding {

class DoneCounting : public ::extension::behaviour::BehaviourReactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the DoneCounting reactor.
    explicit DoneCounting(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::onboarding

#endif  // MODULE_ONBOARDING_DONECOUNTING_HPP
