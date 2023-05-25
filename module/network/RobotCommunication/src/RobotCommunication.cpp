#include "RobotCommunication.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState"
#include "message/localisation/FilteredBall.hpp"

namespace module::network {

    using extension::Configuration;

    RobotCommunication::RobotCommunication(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("RobotCommunication.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RobotCommunication.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            port            = config["port"].as<uint>();
        });

        on<Trigger<FilteredBall>,
           With<Ball>,
           With<Stability>,
           With<WalkCommand>,
           With<Kick>,
           With<Field>,
           With<GameStateProto>,
           With<GlobalConfig>>()
            .then([this](const FilteredBall& ball,
                         const Ball& ballData const Kick& kick,
                         const WalkCommand& walk,
                         const Field& field,
                         const GameStateProto& gameState const GlobalConfig& config) {
                auto teamMessage = std::make_unique<Message>();

                teamMessage->state        = gameState.Phase;
                teamMessage->current_pose = field.covariance;
                eamMessage->walk_command  = walk.command;
                teamMessage->kick_target  = kick.target;

                teamMessage->Robot->player_id  = config.player_id;
                teamMessage->Robot->position   = gameState.Hfw;
                teamMessage->Robot->covariance = gameState.covariance;
                teamMessage->Robot->team       = config.team_id;

                teamMessage->ball->position   = ball.rBTt;
                teamMessage->ball->velocity   = 0;  // velocity is n/a atm
                teamMessage->ball->covariance = ballData.measurement.covariance;

                emit<Scope::UDP>(message);
            });
    }

}  // namespace module::network
