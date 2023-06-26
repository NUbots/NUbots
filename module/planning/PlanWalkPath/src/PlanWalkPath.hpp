#ifndef MODULE_PLANNING_PLANWALKPATH_HPP
#define MODULE_PLANNING_PLANWALKPATH_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::planning {

    class PlanWalkPath : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Maximum walk command velocity magnitude for walking to ball
            double max_translational_velocity_magnitude = 0;
            /// @brief Minimum walk command velocity for walking to ball
            double min_translational_velocity_magnitude = 0;
            /// @brief Crude acceleration, the maximum increment/decrease in walk command velocity per update
            double acceleration = 0;
            /// @brief Region around ball to begin decelerating in
            double approach_radius = 0;
            /// @brief Maximum angular velocity command for walking to ball
            double max_angular_velocity = 0;
            /// @brief Minimum angular velocity command for walking to ball
            double min_angular_velocity = 0;
            /// @brief Rotate on spot walk command angular velocity
            double rotate_velocity = 0;
            /// @brief Rotate on spot walk command forward velocity
            double rotate_velocity_x = 0;
            /// @brief Rotate on spot walk command side velocity
            double rotate_velocity_y = 0;
            /// @brief Pivot ball command angular velocity
            double pivot_ball_velocity = 0;
            /// @brief Pivot ball forward velocity
            double pivot_ball_velocity_x = 0;
            /// @brief Pivot ball side velocity
            double pivot_ball_velocity_y = 0;
        } cfg;

        /// @brief Current magnitude of the translational velocity of the walk command
        double velocity_magnitude = 0;

    public:
        /// @brief Called by the powerplant to build and setup the PlanWalkPath reactor.
        explicit PlanWalkPath(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_PLANWALKPATH_HPP
