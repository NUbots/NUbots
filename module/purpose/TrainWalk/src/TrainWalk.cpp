#include "TrainWalk.hpp"

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/skill/Walk.hpp"
#include "message/support/optimisation/OptimisationCommand.hpp"

#include "utility/rl/ProximalPolicyOptimization.h"


namespace module::purpose {

    using extension::Configuration;

    using Action = message::rl::Action;
    using Reward = message::rl::Reward;
    using State  = message::rl::State;
    using message::input::Sensors;

    using message::skill::Walk;
    using message::support::optimisation::OptimisationCommand;

    TrainWalk::TrainWalk(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("TrainWalk.yaml").then([this](const Configuration& config) {
            // Use configuration here from file TrainWalk.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // ac->to(torch::kF64);
            // ac->normal(0., std);
        });

        on<Startup>().then([this] {
            // Emit walk skill
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d(0.1, 0.0, 0.0)));
        });

        on<Trigger<Sensors>, With<Walk>>().then(
            [this](const Sensors& sensors, const Walk& walk) { log<NUClear::DEBUG>("Construct state"); });

        on<Trigger<State>>().then([this](const State& state) {
            // torch::optim::Adam opt(ac->parameters(), 1e-3);
        });
    }

    // torch::Tensor TrainWalk::calculate_reward(const torch::Tensor& state, torch::Tensor& action) {
    //     // Calculate the reward
    //     return torch::tensor(0.0);
    // }

}  // namespace module::purpose
