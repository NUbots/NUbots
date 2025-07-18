#ifndef MODULE_PURPOSE_PENALTYSHOOTOUT_HPP
#define MODULE_PURPOSE_PENALTYSHOOTOUT_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    class PenaltyShootout : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Whether this robot is the goalie
            bool is_goalie = false;
            /// @brief Distance from the ball at which to perform actions
            double ball_action_distance = 0.5;
            /// @brief Delay before playing
            int startup_delay = 0;
            /// @brief Timeout for searching for the ball
            NUClear::clock::duration ball_search_timeout = std::chrono::seconds(5);
        } cfg;

        /// @brief A high-level update rate for the director tree
        static constexpr size_t BEHAVIOUR_UPDATE_RATE = 10;

    public:
        /// @brief Called by the powerplant to build and setup the PenaltyShootout reactor.
        explicit PenaltyShootout(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_PENALTYSHOOTOUT_HPP
