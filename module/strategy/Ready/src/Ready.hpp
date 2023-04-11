#ifndef MODULE_STRATEGY_READY_HPP
#define MODULE_STRATEGY_READY_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class Ready : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief The amount of time to walk in ready
            NUClear::clock::duration walk_to_ready_time{};
            /// @brief Walk to ready walk command forward velocity
            float walk_to_ready_speed_x = 0;
            /// @brief Walk to ready walk command side velocity
            float walk_to_ready_speed_y = 0;
            /// @brief Walk to ready walk command angular velocity
            float walk_to_ready_rotation = 0;
        } cfg;

        /// @brief Stores the time stamp of when the robot starts walking to the ready position
        NUClear::clock::time_point start_ready_time;

    public:
        /// @brief Called by the powerplant to build and setup the Ready reactor.
        explicit Ready(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_READY_HPP
