#ifndef MODULE_STRATEGY_STARTSAFELY_HPP
#define MODULE_STRATEGY_STARTSAFELY_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class StartSafely : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief How long to wait for the subcontroller to connect
            int start_delay = 0;
            /// @brief How long to tell the servo to move for in seconds
            int move_time = 0;
            /// @brief Gain for the servos
            double servo_gain = 0.0;
            /// @brief Stores the servo targets
            std::map<int, double> servo_targets = {};
        } cfg{};

        /// @brief The time the reactor started
        NUClear::clock::time_point startup_time = NUClear::clock::now();

    public:
        /// @brief Called by the powerplant to build and setup the StartSafely reactor.
        explicit StartSafely(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_STARTSAFELY_HPP
