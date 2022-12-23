#include "TestModule.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdio>

#include "extension/Configuration.hpp"

#include "message/Robocup.hpp"
#include "message/behaviour/KickPlan.hpp"
#include "message/input/GameState.hpp"
#include "message/input/Image.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/motion/WalkCommand.hpp"
#include "message/support/GlobalConfig.hpp"
#include "message/vision/Ball.hpp"
#include "message/vision/Goal.hpp"

namespace module {

    using extension::Configuration;

    using message::Robocup;
    using message::behaviour::KickPlan;
    using message::input::GameState;
    using message::input::Image;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::motion::WalkCommand;
    using message::support::GlobalConfig;

    using Message          = message::Robocup::Message;
    using State            = message::Robocup::State;
    using LocalisationBall = message::localisation::Ball;
    using PenaltyReason    = GameState::Data::PenaltyReason;

    TestModule::TestModule(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("TestModule.yaml").then([this](const Configuration& config) {
            // Use configuration here from file TestModule.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.increment    = config["increment"].as<int>();
            cfg.send_port    = config["send_port"].as<uint>();
            cfg.recieve_port = config["receive_port"].as<uint>();
        });

        on<Every<5, Per<std::chrono::seconds>>,
           With<GlobalConfig>,
           With<Sensors>,
           With<Field>,
           With<LocalisationBall>,
           With<KickPlan>,
           With<GameState>,
           With<WalkCommand>,
           Single>()
            .then([this](const std::shared_ptr<const GlobalConfig>& global,
                         const std::shared_ptr<const Sensors>& sensors,
                         const std::shared_ptr<const Field>& field,
                         const std::shared_ptr<const LocalisationBall>& loc_ball,
                         const std::shared_ptr<const KickPlan>& kick_plan,
                         const std::shared_ptr<const GameState>& game_state,
                         const std::shared_ptr<const WalkCommand>& walk_command) {
                auto packet = std::make_unique<Message>();

                // 1. timestamp
                packet->timestamp = NUClear::clock::now();

                auto pr = game_state->data.self.penalty_reason;

                // 2. game state
                if (pr == PenaltyReason::UNPENALISED) {
                    packet->state = State::UNPENALISED;
                }
                else if (pr >= PenaltyReason::UNKNOWN_PENALTY_REASON && pr <= PenaltyReason::PLAYER_PUSHING) {
                    packet->state = State::PENALISED;
                }
                else {
                    packet->state = State::UNKNOWN_STATE;
                }

                // 3. current pose
                // 3.1 player id

                packet->current_pose.player_id = global->player_id;

                // transformations to field space
                Eigen::Isometry3d Htw(sensors->Htw);

                Eigen::Isometry3d Hfw;
                Eigen::Isometry2d position(field->position);

                Hfw.translation() = Eigen::Vector3d(position.translation().x(), position.translation().y(), 0);
                Hfw.linear() =
                    Eigen::AngleAxisd(Eigen::Rotation2Dd(position.rotation()).angle(), Eigen::Vector3d::UnitZ())
                        .toRotationMatrix();

                Eigen::Isometry3d Hft = Hfw * Htw.inverse();
                Eigen::Vector3d rTFf  = Hft.translation();

                // 3.2 position and covariance
                packet->current_pose.position =
                    Eigen::Vector3f(rTFf.x(), rTFf.y(), Hft.rotation().matrix().eulerAngles(0, 1, 2).z());
                packet->current_pose.covariance = field->covariance.cast<float>();

                // 4. walk command
                // TODO x and y velocity estimations

                Eigen::Vector3f motion_walk_command = walk_command->command.cast<float>();
                packet->walk_command.z()            = motion_walk_command.z();

                // 5 target pose
                packet->target_pose.position.x() = motion_walk_command.x();
                packet->target_pose.position.y() = motion_walk_command.y();
                // TODO extrapolate desired orientation from walk command?

                // 6. kick target
                packet->kick_target = kick_plan->target.cast<float>();

                // 7. ball

                // 7.1 ball position
                Eigen::Vector4d rBWw(loc_ball->position.x(), loc_ball->position.y(), 0.0, 1.0);

                Eigen::Vector4d rBFf = Hfw * rBWw;

                packet->ball.position = Eigen::Vector3f(rBFf.x(), rBFf.y(), 0);

                // 7.3 ball covariance
                Eigen::Matrix3f covariance   = Eigen::Matrix3f::Zero();
                covariance.block<2, 2>(0, 0) = loc_ball->covariance.cast<float>();

                // 7.2 ball velocity
                if (prev_timestamp != std::chrono::time_point<std::chrono::steady_clock>::min()
                    && prev_position != Eigen::Vector3f::Zero()) {
                    auto duration = packet->timestamp - prev_timestamp;
                    auto seconds  = std::chrono::duration_cast<std::chrono::duration<float>>(duration).count();

                    Eigen::Vector3f velocity = (packet->ball.position - prev_position).array() / seconds;

                    packet->ball.velocity = velocity;
                }

                prev_position  = packet->ball.position;
                prev_timestamp = packet->timestamp;

                // 7.3 ball covariance
                packet->ball.covariance = covariance;

                // 8 Other Robots
                // TODO
            });

        // test data
        on<Every<5, Per<std::chrono::seconds>>>().then([this]() {
            auto global       = std::make_unique<GlobalConfig>();
            global->player_id = 69;
            emit(global);

            auto sensors = std::make_unique<Sensors>();
            Eigen::Isometry3d Htw;

            Htw.translation() = Eigen::Vector3d(1, 2, 3);
            Htw.linear()      = Eigen::AngleAxisd(30 * M_PI / 180, Eigen::Vector3d::UnitX()).toRotationMatrix();

            sensors->Htw = Htw.matrix();
            emit(sensors);

            auto field = std::make_unique<Field>();
            Eigen::Isometry2d position;

            position.translation() << 1, 2;
            position.linear() = Eigen::Rotation2Dd(M_PI / 4).toRotationMatrix();
            field->position   = position.matrix();
            emit(field);

            auto loc_ball      = std::make_unique<LocalisationBall>();
            loc_ball->position = Eigen::Vector2d(test, 2);
            test++;
            emit(loc_ball);

            auto kick_plan = std::make_unique<KickPlan>();
            emit(kick_plan);

            auto game_state                      = std::make_unique<GameState>();
            game_state->data.self.penalty_reason = PenaltyReason::REQUEST_FOR_PICKUP;
            emit(game_state);

            auto walk_command     = std::make_unique<WalkCommand>();
            walk_command->command = Eigen::Vector3d(9, 9, 9);
            emit(walk_command);
        });


        //        on<Every<5, Per<std::chrono::seconds>>, With<GameState, WalkCommand, Ball,
        //        Overview>>().then([this](GameState game_state, WalkCommand walk_command, Ball ball, Overview overview)
        //        {
        //            auto packet = std::make_unique<Message>();
        //
        //            packet->timestamp = NUClear::clock::now();
        //
        //            // Set state
        //            PenaltyReason pr = game_state.data.self.penalty_reason;
        //
        //            if (pr == PenaltyReason::UNPENALISED) {
        //                packet->state = State::UNPENALISED;
        //            }
        //            else if (pr >= PenaltyReason::UNKNOWN_PENALTY_REASON && pr <= PenaltyReason::PLAYER_PUSHING) {
        //                packet->state = State::PENALISED;
        //            }
        //            else {
        //                packet->state = State::UNKNOWN_STATE;
        //            }
        //
        //            // current pose
        //            const Eigen::Vector3f& current_position = overview.robot_position;
        //            packet->current_pose.player_id = overview.robot_id;
        //
        //            packet->current_pose.position.x = current_position.x();
        //            packet->current_pose.position.y = current_position.y();
        //
        //            // Set walk command
        //            const Eigen::Vector3f& command = walk_command.command.cast<float>();
        //            packet->walk_command.x = command.x();
        //            packet->walk_command.y = command.y();
        //            packet->walk_command.z = command.z();
        //
        //            // Send ball
        //            const Eigen::Vector2f& position = overview.ball_position;
        //            packet->ball.position.x = position.x();
        //            packet->ball.position.y = position.y();
        //            packet->ball.position.z = 0;
        //
        //            const Eigen::Matrix2f& position_covariance = overview.ball_position_covariance;
        //
        //            std::cout << "ball position y: " << packet->ball.position.y << std::endl;
        //
        //        });
        //
        //        on<Every<5, Per<std::chrono::seconds>>>().then([this](){
        //            // test gamestate
        //            auto game_state = std::make_unique<GameState>();
        //            GameState::Data::Robot new_self = GameState::Data::Robot(1, PenaltyReason::PLAYER_PUSHING,
        //            std::chrono::steady_clock::now()); game_state->data.self = new_self; emit(game_state);
        //            log<NUClear::INFO>("emitted game state!");
        //
        //            // test walkcommand
        //            auto walk_command = std::make_unique<WalkCommand>(
        //                1,
        //                Eigen::Vector3d(1.0, 2.0, 3.0));
        //            emit(walk_command);
        //
        //            // test ball
        //            auto ball = std::make_unique<Ball>();
        //            ball->position = Eigen::Vector2d(2.0,4.0);
        //            emit(ball);
        //                (
        //                Eigen::Vector2d(2.0,4.0),
        //                (
        //                    Eigen::Vector2d(3.0,6.0),
        //                    Eigen::Vector2d(9.0,12.0)
        //                )
        //           );

        // test overview

        //        });

        //        on<Startup>().then([this] {
        //            auto packet = std::make_unique<Message>();
        //            packet->state = State::PENALISED;
        //            emit<Scope::UDP>(packet, INADDR_LOOPBACK, cfg.send_port);
        //        });

        //        on<UDP>(cfg.send_port).then([this](const UDP::Packet& packet) {
        //            log<NUClear::INFO>("received packed triggered");
        //
        //            // Get our packet contents
        //            const auto newPacket = NUClear::util::serialise::Serialise<Message>::deserialise(packet.payload);
        //
        //            log<NUClear::INFO>("received packet converted");
        //
        //            if (newPacket.state == State::PENALISED) {
        //                log<NUClear::INFO>("state found to be Penalised!!!");
        //            }
        //        });
    }
}  // namespace module