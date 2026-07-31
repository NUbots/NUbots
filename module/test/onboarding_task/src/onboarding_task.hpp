#ifndef MODULE_TEST_ONBOARDING_TASK_HPP
#define MODULE_TEST_ONBOARDING_TASK_HPP

#include <nuclear>

namespace module::test {

class onboarding_task : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the onboarding_task reactor.
    explicit onboarding_task(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::test

#endif  // MODULE_TEST_ONBOARDING_TASK_HPP
