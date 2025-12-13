#ifndef MODULE_ONBOARDING_ACTUATOR_HPP
#define MODULE_ONBOARDING_ACTUATOR_HPP

#include <nuclear>

namespace module::onboarding::SumInt {

    class Actuator : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the Actuator reactor.
        explicit Actuator(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::onboarding::SumInt

#endif  // MODULE_ONBOARDING_ACTUATOR_HPP
