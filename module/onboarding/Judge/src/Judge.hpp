#ifndef MODULE_ONBOARDING_JUDGE_HPP
#define MODULE_ONBOARDING_JUDGE_HPP

#include <nuclear>

namespace module::onboarding {

class Judge : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
        int answer = 55;
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Judge reactor.
    explicit Judge(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::onboarding

#endif  // MODULE_ONBOARDING_JUDGE_HPP
