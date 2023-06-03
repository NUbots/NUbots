#include "message/strategy/Defend.hpp"

#include "Defend.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/planning/KickTo.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::strategy {

    using extension::Configuration;
    using DefendTask = message::strategy::Defend;
    using utility::support::Expression;

    Defend::Defend(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Defend.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Defend.yaml
            this->log_level       = config["log_level"].as<NUClear::LogLevel>();
            cfg.defender_position = Eigen::Vector3f(config["defender_position"].as<Expression>());
            cfg.defending_region  = Eigen::Vector4f(config["defending_region"].as<Expression>());

            log<NUClear::DEBUG>("cfg.defender_position ", cfg.defender_position.transpose());
            log<NUClear::DEBUG>("cfg.defending_region ", cfg.defending_region.transpose());
        });

        on<Provide<DefendTask>>().then([this](const DefendTask& defend_task) {
            //
        });
    }

}  // namespace module::strategy
