#include "FieldPlayer.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/Robot.hpp"
#include "message/purpose/Player.hpp"
#include "message/purpose/Purpose.hpp"
#include "message/strategy/FindBall.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/strategy/Who.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/support/GlobalConfig.hpp"

#include "utility/strategy/positioning.hpp"
#include "utility/strategy/soccer_strategy.hpp"

namespace module::purpose {

    using extension::Configuration;

    using FieldPlayerMsg = message::purpose::FieldPlayer;
    using Phase          = message::input::GameState::Phase;

    using message::input::GameState;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::localisation::Robots;
    using message::purpose::Attack;
    using message::purpose::Defend;
    using message::purpose::Purpose;
    using message::purpose::ReadyAttack;
    using message::purpose::SoccerPosition;
    using message::purpose::Support;
    using message::strategy::FindBall;
    using message::strategy::LookAtBall;
    using message::strategy::WalkToFieldPosition;
    using message::strategy::Who;
    using message::support::FieldDescription;
    using message::support::GlobalConfig;

    FieldPlayer::FieldPlayer(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("FieldPlayer.yaml").then([this](const Configuration& config) {
            // Use configuration here from file FieldPlayer.yaml
            this->log_level               = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_threshold            = config["ball_threshold"].as<double>();
            cfg.equidistant_threshold     = config["equidistant_threshold"].as<double>();
            cfg.ball_off_center_threshold = config["ball_off_center_threshold"].as<double>();
            cfg.center_circle_offset      = config["center_circle_offset"].as<double>();
        });

        // PLAYING state
        on<Provide<FieldPlayerMsg>,
           Optional<With<Ball>>,
           Optional<With<Robots>>,
           With<Sensors>,
           With<Field>,
           With<GameState>,
           With<GlobalConfig>,
           With<FieldDescription>,
           When<Phase, std::equal_to, Phase::PLAYING>>()
            .then([this](const std::shared_ptr<const Ball>& ball,
                         const std::shared_ptr<const Robots>& robots,
                         const Sensors& sensors,
                         const Field& field,
                         const GameState& game_state,
                         const GlobalConfig& global_config,
                         const FieldDescription& fd) {
                // Todo determine if we have enough information to play
                // Eg localisation confidence

                // General tasks
                emit<Task>(std::make_unique<FindBall>(), 2);    // Need to know where the ball is
                emit<Task>(std::make_unique<LookAtBall>(), 1);  // Track the ball

                // If there's no ball message, only emit the tasks to find it
                if (ball == nullptr) {
                    // We don't know what we're doing and we're not active
                    emit(std::make_unique<Purpose>(global_config.player_id,
                                                   SoccerPosition::UNKNOWN,
                                                   true,
                                                   false,
                                                   game_state.team.team_colour));
                    return;
                }

                // Make an ignore list with the goalie, if they exist
                std::vector<unsigned int> ignore_ids{};
                // Add inactive robots to the ignore list
                for (const auto& robot : (robots ? robots->robots : std::vector<message::localisation::Robot>{})) {
                    if (!robot.purpose.active) {
                        ignore_ids.push_back(robot.purpose.player_id);
                    }
                }

                // Find who has the ball, if any
                // If there are no robots, use an empty vector
                Who ball_pos = utility::strategy::ball_possession(ball->rBWw,
                                                                  (robots ? *robots : Robots{}),
                                                                  field.Hfw,
                                                                  sensors.Hrw,
                                                                  cfg.ball_threshold,
                                                                  cfg.equidistant_threshold,
                                                                  global_config.player_id,
                                                                  ignore_ids);

                // If we have robots, determine if we are closest to the ball
                // Otherwise assume we are alone and closest by default
                // Only consider the goalie if the ball is in the defending third
                Eigen::Vector3d rBFf     = field.Hfw * ball->rBWw;
                double defending_third_x = fd.dimensions.field_length / 3.0;
                if (robots && rBFf.x() < defending_third_x) {
                    for (const auto& robot : (robots ? robots->robots : std::vector<message::localisation::Robot>{})) {
                        if (robot.purpose.purpose == SoccerPosition::GOALIE) {
                            log<DEBUG>("Ball not in defending third, ignoring goalie for closest to ball");
                            ignore_ids.push_back(robot.purpose.player_id);
                        }
                    }
                }
                const unsigned int closest_to_ball =
                    robots ? utility::strategy::closest_to_ball_on_team(ball->rBWw,
                                                                        *robots,
                                                                        field.Hfw,
                                                                        sensors.Hrw,
                                                                        cfg.equidistant_threshold,
                                                                        global_config.player_id,
                                                                        ignore_ids)
                           : global_config.player_id;
                bool is_closest = closest_to_ball == global_config.player_id;


                // Determine if the game is in a penalty situation
                bool penalty = game_state.mode.value >= GameState::Mode::DIRECT_FREEKICK
                               && game_state.mode.value <= GameState::Mode::THROW_IN;

                // If sub_mode is 0, the robot must freeze for referee ball repositioning
                // If sub_mode is 2, the robot must freeze until the referee calls execute
                if (penalty && (game_state.secondary_state.sub_mode == 0 || game_state.secondary_state.sub_mode == 2)) {
                    log<DEBUG>("We are in a freeze penalty situation, do nothing.");
                    return;
                }

                // Determine if we need to wait for the other team to kick off
                // If the ball moves, it is in play
                bool ball_moved   = (field.Hfw * ball->rBWw).norm() > cfg.ball_off_center_threshold;
                bool kickoff_wait = !ball_moved && !game_state.our_kick_off
                                    && (game_state.secondary_time - NUClear::clock::now()).count() > 0;
                // At this point, if a penalty state is in progress, it must be in sub_mode 1,
                // which is the setup phase where the robot can position to defend or attack.
                bool allowed_to_attack = !(kickoff_wait || penalty);

                // Attack if we are closest BUT we have to be in a situation where we are allowed to attack, eg not in
                // penalty set up phase.
                if (is_closest && allowed_to_attack) {
                    log<DEBUG>("Attack!");
                    emit(std::make_unique<Purpose>(global_config.player_id,
                                                   SoccerPosition::ATTACK,
                                                   true,
                                                   true,
                                                   game_state.team.team_colour));
                    emit<Task>(std::make_unique<Attack>(ball_pos));
                    return;
                }

                // If we are in the best position to attack, but we can't because of the situation, eg penalty
                // positioning or opponent kickoff, then we should stick to a good spot and be ready to attack
                if (is_closest && !allowed_to_attack) {
                    log<DEBUG>("Ready attack!");
                    emit(std::make_unique<Purpose>(global_config.player_id,
                                                   SoccerPosition::ATTACK,
                                                   true,
                                                   true,
                                                   game_state.team.team_colour));
                    emit<Task>(std::make_unique<ReadyAttack>());
                    return;
                }

                // If we can't attack, eg another robot is attacking, we don't want to get in the way.
                // If we are closest to our goals (ignoring the attacking player), then stand back to defend.
                // If there's no robots, assume we are alone and are the furthest back
                // Shouldn't happen, as that should make us the attacker
                // Add closest_to_ball to the ignore list so we don't consider the attacker as the furthest back
                ignore_ids.push_back(closest_to_ball);
                // Add goalies to the ignore list so we don't consider them as the furthest back
                for (const auto& robot : (robots ? robots->robots : std::vector<message::localisation::Robot>{})) {
                    if (robot.purpose.purpose == SoccerPosition::GOALIE) {
                        ignore_ids.push_back(robot.purpose.player_id);
                    }
                }
                bool furthest_back = robots ? utility::strategy::furthest_back(*robots,
                                                                               field.Hfw,
                                                                               sensors.Hrw,
                                                                               cfg.equidistant_threshold,
                                                                               global_config.player_id,
                                                                               ignore_ids)
                                            : true;
                if (furthest_back) {
                    log<DEBUG>("Defend!");
                    emit(std::make_unique<Purpose>(global_config.player_id,
                                                   SoccerPosition::DEFEND,
                                                   true,
                                                   true,
                                                   game_state.team.team_colour));
                    emit<Task>(std::make_unique<Defend>());
                    return;
                }

                // If we're not the attacker, nor are we the robot hanging back to protect in case the opponent takes
                // the ball up towards our goal, we should help out the attacker however makes sense in the situation
                log<DEBUG>("Support!");
                emit<Task>(std::make_unique<Support>());
                emit(std::make_unique<Purpose>(global_config.player_id,
                                               SoccerPosition::SUPPORT,
                                               true,
                                               true,
                                               game_state.team.team_colour));
            });

        // READY state
        on<Provide<FieldPlayerMsg>,
           Optional<With<Robots>>,
           With<FieldDescription>,
           With<Field>,
           With<Sensors>,
           With<GameState>,
           With<GlobalConfig>,
           When<Phase, std::equal_to, Phase::READY>>()
            .then([this](const std::shared_ptr<const Robots>& robots,
                         const FieldDescription& field_desc,
                         const Field& field,
                         const Sensors& sensors,
                         const GameState& game_state,
                         const GlobalConfig& global_config) {
                // Collect up teammates; empty if no one is around
                std::vector<Eigen::Vector3d> teammates{};
                if (robots) {
                    // Collect all teammates in a vector, ignore the goalie
                    for (const auto& robot : robots->robots) {
                        if (robot.teammate && robot.purpose.purpose != SoccerPosition::GOALIE) {
                            teammates.push_back(field.Hfw * robot.rRWw);
                        }
                    }
                }

                // Calculate optimal ready position based on everyone's position
                Eigen::Isometry3d Hfr = utility::strategy::ready_position(field.Hfw,
                                                                          sensors.Hrw,
                                                                          teammates,
                                                                          field_desc,
                                                                          game_state.our_kick_off,
                                                                          cfg.center_circle_offset);
                emit<Task>(std::make_unique<WalkToFieldPosition>(Hfr, true));

                // Send purpose
                emit(std::make_unique<Purpose>(global_config.player_id,
                                               SoccerPosition::UNKNOWN,
                                               true,
                                               true,
                                               game_state.team.team_colour));
            });

        // When not in playing or ready state, send off the team colour and unknown state
        on<Provide<FieldPlayerMsg>, With<GameState>, With<GlobalConfig>>().then(
            [this](const GameState& game_state, const GlobalConfig& global_config) {
                emit(std::make_unique<Purpose>(global_config.player_id,
                                               SoccerPosition::UNKNOWN,
                                               true,
                                               true,
                                               game_state.team.team_colour));
            });
    }
}  // namespace module::purpose
