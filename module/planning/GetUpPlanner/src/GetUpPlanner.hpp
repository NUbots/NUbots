#ifndef MODULE_PLANNING_GETUPPLANNER_HPP
#define MODULE_PLANNING_GETUPPLANNER_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::planning {

    class GetUpPlanner : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief The number of frames we need to be at recovery levels before we trigger
            int count = 30;
            /// @brief The cosine of the angle we need to be at to trigger
            float cos_angle = 0.0f;
            /// @brief The maximum rate the gyro can be moving at to trigger
            float gyro = 0.0f;
            /// @brief The acceleration due to gravity to remove from the accelerometer readings
            float g = 9.8f;
            /// @brief The maximum acceleration we can have to trigger
            float acc = 0.0f;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the GetUpPlanner reactor.
        explicit GetUpPlanner(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief The number of consecutive frames we have been at recovery levels
        int recovery_frames = 0;
    };


}  // namespace module::planning

#endif  // MODULE_PLANNING_GETUPPLANNER_HPP
