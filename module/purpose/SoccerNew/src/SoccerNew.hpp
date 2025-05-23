#ifndef MODULE_SOCCERNEW_HPP
#define MODULE_SOCCERNEW_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    struct EnableIdle {};
    struct DisableIdle {};

    class SoccerNew : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Whether or not to force the robot to ignore GameController and play
            bool force_playing = false;
            /// @brief Delay in seconds before the robot starts playing after button press
            int disable_idle_delay = 0;
            /// @brief Whether or not the player is a goalie
            bool is_goalie = false;
        } cfg;

        /// @brief Idle state of the robot
        bool idle = false;

    public:
        /// @brief Called by the powerplant to build and setup the SoccerNew reactor.
        explicit SoccerNew(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_SOCCERNEW_HPP
