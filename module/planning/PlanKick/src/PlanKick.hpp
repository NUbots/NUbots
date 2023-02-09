#ifndef MODULE_PLANNING_PLANKICK_HPP
#define MODULE_PLANNING_PLANKICK_HPP

#include <extension/Behaviour.hpp>
#include <nuclear>
#include <string>

namespace module::planning {

    class PlanKick : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            float ball_timeout_threshold  = 0.0;
            float ball_distance_threshold = 0.0;
            float ball_angle_threshold    = 0.0;
            float target_angle_threshold  = 0.0;
        } cfg;

        enum KickLeg { AUTO, LEFT, RIGHT } kick_leg{};
        /// @brief True if a Kick task has been requested, false otherwise. Resets to false when the Kick task is Done.
        bool kicking = false;

    public:
        /// @brief Called by the powerplant to build and setup the PlanKick reactor.
        explicit PlanKick(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_PLANKICK_HPP
