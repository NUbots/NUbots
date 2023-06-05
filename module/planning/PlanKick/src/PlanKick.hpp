#ifndef MODULE_PLANNING_PLANKICK_HPP
#define MODULE_PLANNING_PLANKICK_HPP

#include <extension/Behaviour.hpp>
#include <nuclear>
#include <string>

#include "utility/input/LimbID.hpp"

namespace module::planning {

    class PlanKick : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            float ball_timeout_threshold  = 0.0f;
            float ball_distance_threshold = 0.0f;
            float ball_angle_threshold    = 0.0f;
            float target_angle_threshold  = 0.0f;
            utility::input::LimbID kick_leg{};
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the PlanKick reactor.
        explicit PlanKick(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_PLANKICK_HPP
