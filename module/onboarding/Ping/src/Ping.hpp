#ifndef MODULE_ONBOARDING_PING_HPP
#define MODULE_ONBOARDING_PING_HPP

#include <nuclear>

namespace module::onboarding {

    class Ping : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            int num   = 1;
            int total = 1;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the Ping reactor.
        explicit Ping(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::onboarding

#endif  // MODULE_ONBOARDING_PING_HPP
