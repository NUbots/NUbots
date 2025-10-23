#ifndef MODULE_ONBOARDING_JUDGE_HPP
#define MODULE_ONBOARDING_JUDGE_HPP

#include <nuclear>
#include "extension/Behaviour.hpp"



namespace module::onboarding {

class Judge : public ::extension::behaviour::BehaviourReactor {

private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Judge reactor.
    explicit Judge(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::onboarding

#endif  // MODULE_ONBOARDING_JUDGE_HPP
