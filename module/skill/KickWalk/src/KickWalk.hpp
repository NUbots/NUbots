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
            double kick_approach_velocity_x = 0.0;  // Forward velocity during kick (m/s)
            double kick_approach_velocity_y = 0.0;  // Lateral velocity during kick (m/s)
            long new_kick_wait_time         = 0.0;  // Time to wait before the next kick (in seconds)

        } cfg;

        /// @brief Time point when the last kick ended. Used to prevent kick spamming.
        NUClear::clock::time_point last_kick_end = NUClear::clock::now();

    public:
        /// @brief Called by the powerplant to build and setup the KickWalk reactor.
        explicit KickWalk(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_KICKWALK_HPP
