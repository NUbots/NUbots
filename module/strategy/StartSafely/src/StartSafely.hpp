#ifndef MODULE_STRATEGY_STARTSAFELY_HPP
#define MODULE_STRATEGY_STARTSAFELY_HPP

#include <nuclear>

namespace module::strategy {

    class StartSafely : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Allowable error in servo position
            double error = 0.0;
        } cfg;

        /// @brief Stores the servo targets that have been seen
        std::map<int, double> servo_targets = {};

        /// @brief The provider for the StartSafely task
        ReactionHandle provider{};

        /// @brief The reaction that checks if the servos are at their target position
        ReactionHandle position_check{};

    public:
        /// @brief Called by the powerplant to build and setup the StartSafely reactor.
        explicit StartSafely(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_STARTSAFELY_HPP
