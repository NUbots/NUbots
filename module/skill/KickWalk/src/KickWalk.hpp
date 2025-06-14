#ifndef MODULE_SKILL_KICKWALK_HPP
#define MODULE_SKILL_KICKWALK_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

#include "utility/input/LimbID.hpp"

namespace module::skill {
    using utility::input::LimbID;

    class KickWalk : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            double kick_step_length  = 0.15;  // Length of kick step (m)
            double kick_step_height  = 0.08;  // Height of kick step (m)
            double kick_velocity_x   = 0.2;   // Forward velocity during kick (m/s)
            double kick_velocity_y   = 0.05;  // Lateral velocity during kick (m/s)
            double normal_velocity_x = 0.1;   // Normal forward walking velocity (m/s)
            int kick_steps           = 2;     // Number of steps to execute kick over

            // State tracking
            utility::input::LimbID current_kick_leg = utility::input::LimbID::UNKNOWN;
            int kick_step_count                     = 0;
            bool is_kicking                         = false;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the KickWalk reactor.
        explicit KickWalk(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_KICKWALK_HPP
