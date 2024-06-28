#ifndef MODULE_STRATEGY_STARTSAFELY_HPP
#define MODULE_STRATEGY_STARTSAFELY_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

#include "message/actuation/Limbs.hpp"

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
            /// @brief The name of the script to load
            std::string script_name = "";
            /// @brief The maximum time to wait for start safely to complete
            double max_timeout = 0.0;
        } cfg{};

        /// @brief The time on startup
        NUClear::clock::time_point startup_time = NUClear::clock::now();

        /// @brief Stores the servo targets
        std::map<int, double> servo_targets = {};

        /// @brief The script to load
        std::shared_ptr<message::actuation::BodySequence> script = nullptr;

        /// @brief The time to tell the servos to move to
        NUClear::clock::time_point destination_time = NUClear::clock::now();

    public:
        /// @brief Called by the powerplant to build and setup the StartSafely reactor.
        explicit StartSafely(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_STARTSAFELY_HPP
