#ifndef MODULE_STRATEGY_DIVETOBALL_HPP
#define MODULE_STRATEGY_DIVETOBALL_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class DiveToBall : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Distance in meters used as a threshold to start a dive
            double diving_distance_threshold = 0;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the DiveToBall reactor.
        explicit DiveToBall(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_DIVETOBALL_HPP
