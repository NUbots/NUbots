#ifndef MODULE_STRATEGY_STARTSAFELY_HPP
#define MODULE_STRATEGY_STARTSAFELY_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class StartSafely : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief How long to tell the servo to move for in seconds
            int move_time = 0;
            /// @brief Gain for the servos
            double servo_gain = 0.0;
            /// @brief Allowable error for the servos
            double servo_error = 0.0;
            /// @brief Stores the servo targets
            std::map<int, double> servo_targets = {};
        } cfg{};

        /// @brief The time to tell the servos to move to
        NUClear::clock::time_point destination_time = NUClear::clock::now();

    public:
        /// @brief Called by the powerplant to build and setup the StartSafely reactor.
        explicit StartSafely(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_STARTSAFELY_HPP
