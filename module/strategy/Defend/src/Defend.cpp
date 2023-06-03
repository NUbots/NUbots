#include "message/strategy/Defend.hpp"

#include "Defend.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/FilteredBall.hpp"
#include "message/planning/KickTo.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::strategy {

    using extension::Configuration;
    using DefendTask = message::strategy::Defend;
    using utility::support::Expression;
    using Ball = message::localisation::FilteredBall;
    using message::localisation::Field;
    using message::strategy::StandStill;
    using message::strategy::WalkToFieldPosition;

    Defend::Defend(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Defend.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Defend.yaml
            this->log_level      = config["log_level"].as<NUClear::LogLevel>();
            cfg.defending_region = Eigen::Vector4f(config["defending_region"].as<Expression>());

            log<NUClear::DEBUG>("cfg.defending_region ", cfg.defending_region.transpose());
        });

        on<Provide<DefendTask>, Trigger<Ball>, With<Field>>().then(
            [this](const DefendTask& defend_task, const Ball& ball, const Field& field) {
                Eigen::Isometry3f Hfw = Eigen::Isometry3f(field.Hfw.cast<float>());
                Eigen::Vector3f rBFf  = Hfw * ball.rBWw;

                log<NUClear::DEBUG>("rBFf: ", rBFf.transpose());

                // Check if the ball is in the defending region
                if (rBFf.x() > cfg.defending_region(0) && rBFf.x() < cfg.defending_region(1)
                    && rBFf.y() > cfg.defending_region(2) && rBFf.y() < cfg.defending_region(3)) {
                    log<NUClear::DEBUG>("Ball inside of defending region");
                    // Do nothing, play normally
                }
                else {
                    // Calculate the defender position
                    Eigen::Vector3f rDFf = Eigen::Vector3f::Zero();
                    rDFf.x()             = std::clamp(rBFf.x(), cfg.defending_region(0), cfg.defending_region(1));
                    rDFf.y()             = std::clamp(rBFf.y(), cfg.defending_region(2), cfg.defending_region(3));
                    emit<Task>(std::make_unique<WalkToFieldPosition>(Eigen::Vector3f(rDFf.x(), rDFf.y(), 0), -M_PI));
                    log<NUClear::DEBUG>("Ball is outside of defending region");
                }
            });
    }

}  // namespace module::strategy
