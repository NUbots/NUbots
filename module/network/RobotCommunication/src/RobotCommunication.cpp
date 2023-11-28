#include "RobotCommunication.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

#include "extension/Configuration.hpp"
#include "message/skill/Kick.hpp"
#include "message/input/GameState.hpp"
#include "message/input/RoboCup.hpp"
#include "message/localisation/Ball.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/localisation/Field.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/input/Sensors.hpp"
#include "message/support/GlobalConfig.hpp"


namespace module::network {

    // add usings
    using extension::Configuration;
    using message::input::RoboCup;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::input::GameState;
    using message::skill::Kick;
    using message::input::Sensors;
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
                std::tie(listen_handle, std::ignore, std::ignore) =
                    on<UDP::Broadcast, Single>(cfg.receive_port)
                        .then([this](const UDP::Packet& p) {
                            const std::vector<char>& payload = p.payload;

                            RoboCup deserializedData = NUClear::util::serialise::Serialise<RoboCup>::deserialise(payload);
                            log<NUClear::DEBUG>("check received ", deserializedData.current_pose.position.z);

                            std::unique_ptr<RoboCup> dataPtr = std::make_unique<RoboCup>(std::move(deserializedData));
                            emit(dataPtr);
                        });
            }
        });

        // walk command should be updated to director
        // determine way to do this where we don't require a filtered loc_ball trigger (optional triggers, every...)
        on<Every<2, Per<std::chrono::seconds>>, Optional<With<Ball>>, Optional<With<WalkState>>, Optional<With<Kick>>, Optional<With<Sensors>>, Optional<With<Field>>, Optional<With<GameState>>, Optional<With<GlobalConfig>>>()
            .then([this](const std::shared_ptr<const  Ball>& loc_ball,
                        const std::shared_ptr<const  WalkState>& walk_state,
                        const std::shared_ptr<const  Kick>& kick,
                        const std::shared_ptr<const  Sensors>& sensors,
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

                Eigen::Isometry3d Hfw;

                // Current pose (Position, orientation, and covariance of the player on the field)
                if (config) {
                    msg->current_pose.player_id = config->player_id;

                    if (sensors) {
                        // Get our world transform
                        Eigen::Isometry3d Htw(sensors->Htw);

                        // If we have field information
                        if (field) {
                            // Transform the field state into Hfw
                            Eigen::Isometry3d Hfw = Eigen::Isometry3d(field->Hfw);

                            // Get our torso in field space
                            Eigen::Isometry3d Hft = Hfw * Htw.inverse();
                            Eigen::Vector3d rTFf = Hft.translation();

                            // Store our position from field to torso
                            msg->current_pose.position.x = rTFf.x();
                            msg->current_pose.position.y = rTFf.y();
                            msg->current_pose.position.z = Hft.rotation().matrix().eulerAngles(0, 1, 2).z();

                            Eigen::Matrix<float, 3, 3> covariance = field->covariance.cast<float>();

                            msg->current_pose.covariance = {
                                { covariance(0, 0), covariance(1, 0), covariance(2, 0) },
                                { covariance(0, 1), covariance(1, 1), covariance(2, 1) },
                                { covariance(0, 2), covariance(1, 2), covariance(2, 2) }
                            };

                            msg->current_pose.team = config->team_id;

                            // Debug to deletePENALTY_REASON
                            log<NUClear::DEBUG>("check sent ", msg->current_pose.position.z);
                            // TODO: maybe test covariance here
                        }
                    }
                }

                // Walk command
                if (walk_state != nullptr) {
                    msg->walk_command = {
                        static_cast<float>(walk_state->velocity_target.x()),
                        static_cast<float>(walk_state->velocity_target.y()),
                        static_cast<float>(walk_state->velocity_target.z())
                    };
                }

                // TODO: target pose

                // Kick target
                if (kick) {
                    message::input::fvec2 kick_target;
                    kick_target.x = kick->target.x();
                    kick_target.y = kick->target.y();
                    msg->kick_target = kick_target;
                    // log<NUClear::DEBUG>("kick_target ", kick_target.x);
                }

                // Ball
                if (loc_ball) {
                    // convert position of ball from world to field space
                    Eigen::Vector3d rBWw = loc_ball->rBWw;
                    Eigen::Vector3d rBFf = Hfw * rBWw;

                    // Store our position from field to ball
                    msg->ball.position.x = rBFf.x();
                    msg->ball.position.y = rBFf.y();
                    msg->ball.position.z = rBFf.z();

                    //TODO: Check if correct, copied from Overview
                    Eigen::Matrix<float, 3, 3> covariance = loc_ball->covariance.block(0, 0, 3, 3).cast<float>();
                    msg->ball.covariance = {
                        { covariance(0, 0), covariance(1, 0), covariance(2, 0) },
                        { covariance(0, 1), covariance(1, 1), covariance(2, 1) },
                        { covariance(0, 2), covariance(1, 2), covariance(2, 2) }
                    };

                    msg->ball.velocity = {static_cast<float>(loc_ball->vBw.x()), static_cast<float>(loc_ball->vBw.y()), static_cast<float>(loc_ball->vBw.z())};
                    // log<NUClear::DEBUG>("Ball velocity vBw: ", loc_ball->vBw);
                }

                // TODO: Robots. Where the robot thinks the other robots are. This doesn't exist yet.

                emit<Scope::UDP>(msg, cfg.broadcast_ip, cfg.send_port);
            }
        );
    }


}  // namespace module::network
