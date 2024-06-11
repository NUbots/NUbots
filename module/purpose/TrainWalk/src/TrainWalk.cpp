#include "TrainWalk.hpp"

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/rl/rl.hpp"
#include "message/skill/Walk.hpp"
#include "message/support/optimisation/OptimisationCommand.hpp"


namespace module::purpose {

    using extension::Configuration;

    using State  = message::input::Sensors;
    using Action = message::rl::FeedForwardPoseAction;
    using Reward = message::rl::Reward;

    using message::skill::Walk;
    using message::support::optimisation::OptimisationCommand;

    TrainWalk::TrainWalk(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("TrainWalk.yaml").then([this](const Configuration& config) {
            // Use configuration here from file TrainWalk.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this] {
            // Emit start policy State, Action, Reward
        });

        // Calculate the reward
        on<Trigger<State>, With<Action>>().then([this](const State& state, const Action& action) {
            // TODO: Calculate the reward
            double reward_value = 0.0;

            emit(std::make_unique<Reward>(reward_value));
        });

        on<Last<BUFFER_SIZE, Trigger<State, Action, Reward>>>().then(
            [this](const std::list<std::shared_ptr<const State>>& state_buffer,
                   const std::list<std::shared_ptr<const Action>>& action_buffer,
                   const std::list<std::shared_ptr<const Reward>>& reward_buffer) {
                // Create tensors for the states, actions and rewards
                std::vector<torch::Tensor> states;
                std::vector<torch::Tensor> actions;
                std::vector<torch::Tensor> rewards;
                for (int i = 0; i < BUFFER_SIZE; i++) {
                }

                // Perform PPO update
            });
    }

}  // namespace module::purpose
