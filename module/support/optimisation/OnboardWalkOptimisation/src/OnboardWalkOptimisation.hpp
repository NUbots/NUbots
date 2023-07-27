#ifndef MODULE_SUPPORT_OPTIMISATION_ONBOARDWALKOPTIMISATION_HPP
#define MODULE_SUPPORT_OPTIMISATION_ONBOARDWALKOPTIMISATION_HPP

#include <nuclear>

namespace module::support::optimisation {

class OnboardWalkOptimisation : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the OnboardWalkOptimisation reactor.
    explicit OnboardWalkOptimisation(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::support::optimisation

#endif  // MODULE_SUPPORT_OPTIMISATION_ONBOARDWALKOPTIMISATION_HPP
