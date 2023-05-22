#include "Overview.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "extension/Configuration.hpp"

#include "message/behaviour/Behaviour.hpp"
#include "message/behaviour/KickPlan.hpp"
#include "message/input/GameState.hpp"
#include "message/input/Image.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/motion/WalkCommand.hpp"
#include "message/support/GlobalConfig.hpp"
#include "message/support/nusight/Overview.hpp"
#include "message/vision/Ball.hpp"
#include "message/vision/Goal.hpp"

namespace module::output {

    using extension::Configuration;
    using message::behaviour::Behaviour;
    using message::behaviour::KickPlan;
    using message::input::GameState;
    using message::input::Image;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::motion::WalkCommand;
    using message::support::GlobalConfig;
    using NUClear::message::CommandLineArguments;

    using LocalisationBall = message::localisation::Ball;
    using VisionBalls      = message::vision::Balls;
    using VisionGoals      = message::vision::Goals;
    using OverviewMsg      = message::support::nusight::Overview;

    Overview::Overview(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Overview.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file Overview.yaml

            // clang-format off
            auto lvl = cfg["log_level"].as<std::string>();
            if (lvl == "TRACE") { this->log_level = NUClear::TRACE; }
            else if (lvl == "DEBUG") { this->log_level = NUClear::DEBUG; }
            else if (lvl == "INFO") { this->log_level = NUClear::INFO; }
            else if (lvl == "WARN") { this->log_level = NUClear::WARN; }
            else if (lvl == "ERROR") { this->log_level = NUClear::ERROR; }
            else if (lvl == "FATAL") { this->log_level = NUClear::FATAL; }
            // clang-format on
        });


        on<Every<2, Per<std::chrono::seconds>>,
           Optional<With<GlobalConfig>>,
           Optional<With<CommandLineArguments>>,
           Optional<With<Sensors>>,
           Optional<With<Behaviour::State>>,
           Optional<With<Field>>,
           Optional<With<LocalisationBall>>,
           Optional<With<KickPlan>>,
           Optional<With<GameState>>,
           Optional<With<WalkCommand>>,
           Single,
           Priority::LOW>()
            .then([this](const std::shared_ptr<const GlobalConfig>& global,
                         const std::shared_ptr<const CommandLineArguments>& cli,
                         const std::shared_ptr<const Sensors>& sensors,
                         const std::shared_ptr<const Behaviour::State>& behaviour_state,
                         const std::shared_ptr<const Field>& field,
                         const std::shared_ptr<const LocalisationBall>& loc_ball,
                         const std::shared_ptr<const KickPlan>& kick_plan,
                         const std::shared_ptr<const GameState>& game_state,
                         const std::shared_ptr<const WalkCommand>& walk_command) {
                auto msg = std::make_unique<OverviewMsg>();

                // Set properties
                msg->timestamp       = NUClear::clock::now();
                msg->robot_id        = global ? global->player_id : 0;
                msg->role_name       = cli ? cli->at(0) : "";
                msg->battery         = sensors ? sensors->battery : 0;
                msg->voltage         = sensors ? sensors->voltage : 0;
                msg->behaviour_state = behaviour_state ? msg->behaviour_state : Behaviour::State(0);

                if (sensors) {
                    // Get our world transform
                    Eigen::Isometry3d Htw(sensors->Htw);

                    // If we have field information
                    if (field) {
                        // Transform the field state into Hfw
                        Eigen::Isometry3d Hfw(field->Hfw);

                        // Get our torso in field space
                        Eigen::Isometry3d Hft = Hfw * Htw.inverse();
                        Eigen::Vector3d rTFf  = Hft.translation();

                        // Store our position from field to torso
                        msg->robot_position =
                            Eigen::Vector3f(rTFf.x(), rTFf.y(), Hft.rotation().matrix().eulerAngles(0, 1, 2).z());
                        msg->robot_position_covariance = field->covariance.cast<float>();

                        if (loc_ball) {
                            // Get our ball in field space
                            Eigen::Vector4d rBWw(loc_ball->position.x(), loc_ball->position.y(), 0.0, 1.0);
                            Eigen::Vector4d rBFf = Hfw * rBWw;

                            // Store our position from field to ball
                            msg->ball_position            = Eigen::Vector2f(rBFf.x(), rBFf.y());
                            msg->ball_position_covariance = loc_ball->covariance.cast<float>();
                        }
                    }
                }

                if (kick_plan) {
                    msg->kick_target = kick_plan->target.cast<float>();
                }

                // Set our game mode properties
                msg->game_mode  = game_state ? game_state->data.mode : GameState::Data::Mode(0);
                msg->game_phase = game_state ? game_state->data.phase : GameState::Data::Phase(0);
                msg->penalty_reason =
                    game_state ? game_state->data.self.penalty_reason : GameState::Data::PenaltyReason(0);

                // Set our last seen times
                msg->last_camera_image = last_camera_image;
                msg->last_camera_image = last_seen_ball;
                msg->last_camera_image = last_seen_goal;

                // Set our walk command
                if (walk_command) {
                    msg->walk_command = walk_command->command.cast<float>();
                }
                else {
                    msg->walk_command = Eigen::Vector3f::Zero();
                }

                emit(msg);
            });


        on<Trigger<Image>, Single, Priority::LOW>().then([this] { last_camera_image = NUClear::clock::now(); });

        on<Trigger<VisionBalls>, Single, Priority::LOW>().then([this](const VisionBalls& balls) {
            if (!balls.balls.empty()) {
                last_seen_ball = NUClear::clock::now();
            }
        });

        on<Trigger<VisionGoals>, Single, Priority::LOW>().then([this](const VisionGoals& goals) {
            if (!goals.goals.empty()) {
                last_seen_goal = NUClear::clock::now();
            }
        });
    }

}  // namespace module::output
