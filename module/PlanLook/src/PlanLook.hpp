#ifndef MODULE_PLANLOOK_HPP
#define MODULE_PLANLOOK_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module {

class PlanLook : public ::extension::behaviour::BehaviourReactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the PlanLook reactor.
    explicit PlanLook(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module

#endif  // MODULE_PLANLOOK_HPP
