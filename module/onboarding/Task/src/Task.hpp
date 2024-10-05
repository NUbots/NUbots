#ifndef MODULE_ONBOARDING_TASK_HPP
#define MODULE_ONBOARDING_TASK_HPP

#include <nuclear>

namespace module::onboarding {

class Task : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Task reactor.
    explicit Task(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::onboarding

#endif  // MODULE_ONBOARDING_TASK_HPP
