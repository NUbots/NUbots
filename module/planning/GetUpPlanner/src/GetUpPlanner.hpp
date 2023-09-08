#ifndef MODULE_PLANNING_GETUPPLANNER_HPP
#define MODULE_PLANNING_GETUPPLANNER_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::planning {

    class GetUpPlanner : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Threshold angle for executing getup, between torso z axis and world z axis
            float fallen_angle = 0.0f;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the GetUpPlanner reactor.
        explicit GetUpPlanner(std::unique_ptr<NUClear::Environment> environment);
    };


}  // namespace module::planning

#endif  // MODULE_PLANNING_GETUPPLANNER_HPP
