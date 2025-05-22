#include "Attack.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/purpose/Player.hpp"
#include "message/strategy/FindBall.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/Who.hpp"

namespace module::purpose {

    using extension::Configuration;

    using AttackMsg = message::purpose::Attack;
    using message::strategy::FindBall;
    using message::strategy::LookAtBall;
    using message::strategy::TackleBall;
    using message::strategy::WalkToKickBall;
    using message::strategy::Who;

    Attack::Attack(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Attack.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Attack.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<AttackMsg>>().then([this](const AttackMsg& attack) {
            // General tasks
            emit<Task>(std::make_unique<FindBall>(), 1);    // Need to know where the ball is
            emit<Task>(std::make_unique<LookAtBall>(), 2);  // Track the ball

            // In this state, either we have the ball or we are the closest to getting the ball and should go for it
            // If the opponent has the ball, we need to tackle it from them
            if (attack.ball_pos == message::strategy::Who::OPPONENT) {
                emit<Task>(std::make_unique<TackleBall>(), 3);  // Tackle the ball from the opponent
                return;
            }
            else {
                // Try to walk to the ball and align towards opponents goal
                emit<Task>(std::make_unique<WalkToKickBall>(), 3);
            }
        });
    }

}  // namespace module::purpose
