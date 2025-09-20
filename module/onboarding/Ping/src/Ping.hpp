#ifndef MODULE_ONBOARDING_PING_HPP
#define MODULE_ONBOARDING_PING_HPP

#include <nuclear>

namespace module::onboarding {

    class Ping : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
        } cfg;

        /// @brief Counts the number of Pong messages received
        std::int32_t message_count = 0;

    public:
        /// @brief Called by the powerplant to build and setup the Ping reactor.
        explicit Ping(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::onboarding

#endif  // MODULE_ONBOARDING_PING_HPP
