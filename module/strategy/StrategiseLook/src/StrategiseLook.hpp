#ifndef MODULE_STRATEGY_STRATEGISELOOK_HPP
#define MODULE_STRATEGY_STRATEGISELOOK_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class StrategiseLook : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief How long before the ball measurement is too old and we have nothing to look at
            NUClear::clock::duration ball_search_timeout{};
            /// @brief How long before the goal measurement is too old and we have nothing to look at
            NUClear::clock::duration goal_search_timeout{};
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the StrategiseLook reactor.
        explicit StrategiseLook(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_STRATEGISELOOK_HPP
