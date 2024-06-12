#ifndef MODULE_PURPOSE_TRAINWALK_HPP
#define MODULE_PURPOSE_TRAINWALK_HPP

#include <nuclear>
#include <torch/torch.h>

#include "extension/Behaviour.hpp"

#include "message/rl/rl.hpp"

// #include "utility/rl/Models.h"
// #include "utility/rl/PPO.hpp"
// #include "utility/rl/ProximalPolicyOptimization.h"
// #include "utility/rl/RLAgent.hpp"
// #include "utility/rl/TestEnvironment.h"

namespace module::purpose {

    class TrainWalk : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            uint n_iter          = 10000;
            uint n_steps         = 2048;
            uint n_epochs        = 15;
            uint mini_batch_size = 512;
            uint ppo_epochs      = 4;
            double beta          = 1e-3;
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
        static const int N_STATES = 6;  // 52;

        /// @brief Buffer size for storing the states and actions
        static const int BUFFER_SIZE = 5;

        /// @brief Agent for training the walk
        // utility::rl::RLAgent agent{N_STATES, N_ACTIONS};

        // VT states;
        // VT actions;
        // VT rewards;
        // VT dones;

        // VT log_probs;
        // VT returns;
        // VT values;

        // // Model.
        // uint n_in  = 4;
        // uint n_out = 2;
        // double std = 2e-2;

        // ActorCritic ac = ActorCritic(n_in, n_out, std);


        // Counter.
        uint c = 0;

        // Average reward.
        double best_avg_reward = 0.;
        double avg_reward      = 0.;


        /// @brief Compute reward for the current state and action
        /// @param state The current state
        /// @param action The current action
        /// @return The reward for the current state and action
        // torch::Tensor calculate_reward(const torch::Tensor& state, torch::Tensor& action);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_TRAINWALK_HPP
