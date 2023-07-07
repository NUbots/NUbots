#ifndef MODULE_STRATEGY_WALKTOFIELDPOSITION_HPP
#define MODULE_STRATEGY_WALKTOFIELDPOSITION_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class WalkToFieldPosition : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Radius to begin aligning with desired field heading
            double align_radius = 0.0f;
            /// @brief Tolerance for stopping at the field position
            double stop_tolerance = 0.0f;
            /// @brief Tolerance for resuming walking to the field position if stopped
            double resume_tolerance = 0.0f;
        } cfg;

        /// @brief Whether or not we are stopped
        bool stopped = false;

    public:
        /// @brief Called by the powerplant to build and setup the WalkToFieldPosition reactor.
        explicit WalkToFieldPosition(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_WALKTOFIELDPOSITION_HPP
