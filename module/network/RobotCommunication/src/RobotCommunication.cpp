#include "RobotCommunication.hpp"

#include "extension/Configuration.hpp"

#include "message/inpiut/RoboCup.hpp"
#include "message/input/GameState"
#include "message/localisation/FilteredBall.hpp"

namespace module::network {

    using extension::Configuration;
    using message::input::RoboCup;

    RobotCommunication::RobotCommunication(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("RobotCommunication.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RobotCommunication.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            send_port       = config["send_port"].as<uint>();
            broadcast_ip    = config["broadcast_ip"].as<uint>();

            // If we are changing ports (the port starts at 0 so this should start it the first time)
            if (config["receive_port"].as<uint>() != receive_port) {
                // If we have an old binding, then unbind it
                // The port starts at 0 so this should work
                if (recieve_port != 0) {
                    listenHandle.unbind();
                }

                receive_port = config["receive_port"].as<uint>();

                // Bind our new handle
                std::tie(listen_handle, std::ignore, std::ignore) =
                    on<UDP::Broadcast, Single>(port).then([this](const UDP::Packet& p) {
                        emit(NUClear::util::serialise::Serialise<SensorMeasurements>::deserialise(p));
                    });
            }
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
                         const GameState& game_state,
                         const GlobalConfig& config) {
                auto msg = std::make_unique<RoboCup>();

                msg->state               = game_state.Phase;
                msg->current_pose        = field.covariance;
                eamMessage->walk_command = walk.command;
                msg->kick_target         = kick.target;

                msg->Robot->player_id  = config.player_id;
                msg->Robot->position   = game_state.Hfw;
                msg->Robot->covariance = game_state.covariance;
                msg->Robot->team       = config.team_id;

                msg->ball->position   = ball.rBTt;
                msg->ball->velocity   = 0;  // velocity is n/a atm
                msg->ball->covariance = ballData.measurement.covariance;

                emit<Scope::UDP>(msg, broadcast_ip, send_port);
            });
    }

}  // namespace module::network
