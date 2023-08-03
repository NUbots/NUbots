#ifndef MODULE_SUPPORT_OPTIMISATION_ONBOARDWALKOPTIMISATION_HPP
#define MODULE_SUPPORT_OPTIMISATION_ONBOARDWALKOPTIMISATION_HPP

#include <nuclear>

namespace module::support::optimisation {

class OnboardWalkOptimisation : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

    /// @brief When set, the next ActuatorRequests will set the reset world command to end the simulation.
    bool reset_simulation_world = false;

    /// @brief When set, the next ActuatorRequests will set the reset time command to end the simulation.
    bool reset_simulation_time = false;

    /// @brief When set, the next ActuatorRequests will set the terminate command to end the simulation.
    bool terminate_simulation = false;

public:
    /// @brief Called by the powerplant to build and setup the OnboardWalkOptimisation reactor.
    explicit OnboardWalkOptimisation(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::support::optimisation

#endif  // MODULE_SUPPORT_OPTIMISATION_ONBOARDWALKOPTIMISATION_HPP
