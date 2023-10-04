#include "RobotCommunication.hpp"

#include "extension/Configuration.hpp"
#include "message/skill/Kick.hpp"
#include "message/input/GameState.hpp"
#include "message/input/RoboCup.hpp"
#include "message/localisation/Ball.hpp"
#include "message/behaviour/state/WalkState.hpp"
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
    using message::behaviour::state::WalkState;
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
        });

        // walk command should be updated to director
        // determine way to do this where we don't require a filtered ball trigger (optional triggers, every...)
        on<Every<2, Per<std::chrono::seconds>>, Optional<With<Ball>>, Optional<With<WalkState>>, Optional<With<Kick>>, Optional<With<Field>>, Optional<With<GameState>>, Optional<With<GlobalConfig>>>()
            .then([this](const std::shared_ptr<const  Ball>& ball,
                         const std::shared_ptr<const  WalkState>& walk_state,
                         const std::shared_ptr<const  Kick>& kick,
                         const std::shared_ptr<const  Field>& field,
                         const std::shared_ptr<const  GameState>& game_state,
                         const std::shared_ptr<const  GlobalConfig>& config) {

                log<NUClear::DEBUG>("Send robocup msg!");



                // log<NUClear::DEBUG>("game_state->data.self.penalty_reason: ", game_state->data.self.penalty_reason);
                int penaltyReason = game_state->data.self.penalty_reason;

                int phase = game_state->data.phase;
                // log<NUClear::DEBUG>("game_state->data.phase: ", game_state->data.phase);


                auto msg = std::make_unique<RoboCup>();


                // Timestamp
                msg->timestamp = NUClear::clock::now();

                // State
                int state = 0;

                switch (penaltyReason) {
                    case 0: state = 0; break;
                    case 1: state = 1; break;
                    default: state = 2; break;
                }

                // Current pose

                // Walk_state command
                if (walk_state != nullptr) {
                    msg->walk_command = {static_cast<float>(walk_state->velocity_target.x()), static_cast<float>(walk_state->velocity_target.y()), static_cast<float>(walk_state->velocity_target.z())};
                    log<NUClear::DEBUG>("walk_state command x", msg->walk_command.x);
                    log<NUClear::DEBUG>("walk_state command y", msg->walk_command.y);
                }

                // Target pose

                // Kick target
                if (kick) {
                    message::input::fvec2 kick_target;
                    kick_target.x = kick->target.x();
                    kick_target.y = kick->target.y();
                    msg->kick_target = kick_target;
                    log<NUClear::DEBUG>("kick_target ", kick_target.x);
                }

                // Robots
                if (robot != nullptr) {
                    // msg->Robot.player_id  = config.player_id;
                    // msg->Robot->position   = game_state.Hfw;
                    // msg->Robot->covariance = game_state.covariance;
                    // msg->Robot->team       = config.team_id;
                }

                // Ball
                if(ball != nullptr) {
                    msg->ball.position = {static_cast<float>(ball->rBWw.x()), static_cast<float>(ball->rBWw.y()), static_cast<float>(ball->rBWw.z())};
                    msg->ball.velocity = {static_cast<float>(ball->vBw.x()), static_cast<float>(ball->vBw.y()), static_cast<float>(ball->vBw.z())};
                    // msg->ball.covariance =  {
                    //     static_cast<float>(ball->covariance[0][0]),
                    //     static_cast<float>(ball->covariance[1][0]),
                    //     static_cast<float>(ball->covariance[2][0])
                    // };


                    float position = msg->ball.position.x; //TODO: I am proof of concept delete me ~
                    log<NUClear::DEBUG>("Ball position rBWw: ", position);
                    log<NUClear::DEBUG>("Ball velocity vBw: ", ball->vBw);
                    log<NUClear::DEBUG>("Ball covariance: ", ball->covariance);
                }

                // msg->walk_command = walk->velocity_target;
                // log<NUClear::DEBUG>("walk command: ", walk->velocity_target);mand;
                // msg->kick_target  = kick.target;


                emit<Scope::UDP>(msg, cfg.broadcast_ip, cfg.send_port);
            });
    }

}  // namespace module::network
