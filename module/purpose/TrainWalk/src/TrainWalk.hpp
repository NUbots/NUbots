#ifndef MODULE_PURPOSE_TRAINWALK_HPP
#define MODULE_PURPOSE_TRAINWALK_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

#include "utility/rl/PPO.hpp"

namespace module::purpose {

    class TrainWalk : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the TrainWalk reactor.
        explicit TrainWalk(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Number of actions
        /// left foot pose 6
        /// right foot pose 6
        static const int N_ACTIONS = 12;

        /// @brief Number of states
        /// torso position 3
        /// torso orientation 3
        /// torso velocity 3
        /// torso angular velocity 3
        /// joint angles 12
        /// joint velocities 12
        /// left foot pose 6
        /// right foot pose 6
        /// walk phase 1
        /// reference velocity 3
        static const int N_STATES = 52;

        /// @brief Buffer size for storing the states and actions
        static const int BUFFER_SIZE = 2;


        /// @brief PPO agent for training the walk
        utility::rl::PPOAgent ppo_agent{N_STATES, N_ACTIONS};
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_TRAINWALK_HPP
