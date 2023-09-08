#include "RobotCommunication.hpp"

#include "extension/Configuration.hpp"
#include "message/skill/Kick.hpp"
#include "message/input/GameState.hpp"
#include "message/input/RoboCup.hpp"
#include "message/localisation/Ball.hpp"
#include "message/skill/Walk.hpp"
#include "message/localisation/Field.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/support/GlobalConfig.hpp"

// add includes

namespace module::network {

    // add usings
    using extension::Configuration;
    using message::input::RoboCup;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::input::GameState;
    using message::skill::Kick;
    using message::platform::webots::SensorMeasurements;
    using message::skill::Walk;
    using message::support::GlobalConfig;

    RobotCommunication::RobotCommunication(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("RobotCommunication.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RobotCommunication.yaml
            log_level = config["log_level"].as<NUClear::LogLevel>();

            // need to determine send and receive ports
            cfg.send_port = config["send_port"].as<uint>();
            // need to determine broadcast ip
            cfg.broadcast_ip = config["broadcast_ip"].as<std::string>("");

            // If we are changing ports (the port starts at 0 so this should start it the first time)
            if (config["receive_port"].as<uint>() != cfg.receive_port) {
                // If we have an old binding, then unbind it
                // The port starts at 0 so this should work
                if (cfg.receive_port != 0) {
                    listen_handle.unbind();
                }

                cfg.receive_port = config["receive_port"].as<uint>();

                // Bind our new handle
                // TODO: check send_port is the correct thing to use here
                // std::tie(listen_handle, std::ignore, std::ignore) =
                //     on<UDP::Broadcast, Single>(cfg.send_port).then([this](const UDP::Packet& p) {
                //         emit(NUClear::util::serialise::Serialise<SensorMeasurements>::deserialise(p));
                //     });
            }

            log<NUClear::DEBUG>("uoooo,dpwomwod");
        });

        // walk command should be updated to director
        // determine way to do this where we don't require a filtered ball trigger (optional triggers, every...)
        on<Every<2, Per<std::chrono::seconds>>, Optional<With<Ball>>, Optional<With<Walk>>, Optional<With<Kick>>, Optional<With<Field>>, Optional<With<GameState>>, Optional<With<GlobalConfig>>>()
            .then([this](const std::shared_ptr<const  Ball>& ball,
                         const std::shared_ptr<const  Walk>& walk,
                         const std::shared_ptr<const  Kick>& kick,
                         const std::shared_ptr<const  Field>& field,
                         const std::shared_ptr<const  GameState>& game_state,
                         const std::shared_ptr<const  GlobalConfig>& config) {

                log<NUClear::DEBUG>("Send robocup msg!");

                if(ball != nullptr) {
                    log<NUClear::DEBUG>("Ball position rBWw: ", ball->rBWw);
                }


                log<NUClear::DEBUG>("game_state->data.self.penalty_reason: ", game_state->data.self.penalty_reason);


                auto msg = std::make_unique<RoboCup>();

                // msg->state        = game_state.phase;

                // Timestamp
                msg->timestamp = NUClear::clock::now();

                // State TODO: Figure out <here> from game_state
                // msg->state = <here>;

                log<NUClear::DEBUG>("State: ", msg->state);
                // Current pose

                // Walk command

                // Target pose

                // Kick target

                // Ball

                // Robots


                // msg->current_pose = field.covariance;
                // // msg->walk_command = walk.command;
                // msg->kick_target  = kick.target;

                // msg->Robot->player_id  = config.player_id;
                // msg->Robot->position   = game_state.Hfw;
                // msg->Robot->covariance = game_state.covariance;
                // msg->Robot->team       = config.team_id;

                // msg->ball->position = ball.rBTt;
                // todo: should add covariance and velocity to the ball message
                // msg->ball->velocity   = 0;  // velocity is n/a atm
                // msg->ball->covariance = ballData.measurement.covariance;

                emit<Scope::UDP>(msg, cfg.broadcast_ip, cfg.send_port);
            });
    }

}  // namespace module::network
