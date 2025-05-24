#include "FieldPlayer.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/Robot.hpp"
#include "message/purpose/Player.hpp"
#include "message/strategy/FindBall.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/strategy/Who.hpp"
#include "message/support/GlobalConfig.hpp"

#include "utility/strategy/soccer_strategy.hpp"

namespace module::purpose {

    using extension::Configuration;

    using FieldPlayerMsg = message::purpose::FieldPlayer;
    using Phase          = message::input::GameState::Phase;

    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::localisation::Robots;
    using message::purpose::Attack;
    using message::purpose::Defend;
    using message::purpose::ReadyAttack;
    using message::purpose::Support;
    using message::strategy::FindBall;
    using message::strategy::StandStill;
    using message::strategy::WalkToFieldPosition;
    using message::strategy::Who;
    using message::support::GlobalConfig;


    FieldPlayer::FieldPlayer(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("FieldPlayer.yaml").then([this](const Configuration& config) {
            // Use configuration here from file FieldPlayer.yaml
            this->log_level           = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_threshold        = config["ball_threshold"].as<double>();
            cfg.equidistant_threshold = config["equidistant_threshold"].as<double>();
        });

        // PLAYING state
        on<Provide<FieldPlayerMsg>,
           Optional<With<Ball>>,
           Optional<With<Robots>>,
           With<Sensors>,
           With<Field>,
           With<GlobalConfig>,
           When<Phase, std::equal_to, Phase::PLAYING>>()
            .then([this](const std::shared_ptr<const Ball>& ball,
                         const std::shared_ptr<const Robots>& robots,
                         const Sensors& sensors,
                         const Field& field,
                         const GlobalConfig& global_config) {
                // emit<Task>(std::make_unique<Defend>());
                // return;
                // log<INFO>("Playing as a FieldPlayer");

                // Todo determine if we have enough information to play
                // Eg localisation confidence

                // If we don't know where the ball is, look for it
                if (!ball) {
                    emit<Task>(std::make_unique<FindBall>());
                    return;
                }

                // If we have robots, determine if we are closest to the ball
                // Otherwise assume we are alone and closest by default
                bool closest_to_ball_on_team =
                    robots ? utility::strategy::closest_to_ball_on_team(ball->rBWw,
                                                                        *robots,
                                                                        field.Hfw,
                                                                        sensors.Hrw,
                                                                        cfg.equidistant_threshold,
                                                                        global_config.player_id)
                           : true;
                // If there are no robots, use an empty vector (might still be self or none)
                Who ball_pos = utility::strategy::ball_possession(ball->rBWw,
                                                                  (robots ? *robots : Robots{}),
                                                                  field.Hfw,
                                                                  sensors.Hrw,
                                                                  cfg.ball_threshold,
                                                                  cfg.equidistant_threshold,
                                                                  global_config.player_id);

                // Todo Check if we can attack
                bool allowed_to_attack = true;

                log<DEBUG>("Ball possession:",
                           ball_pos,
                           " closest to ball?",
                           closest_to_ball_on_team,
                           " allowed to attack?",
                           allowed_to_attack);

                // Todo: if another robot/s within a threshold distance, the lowest robot id should be the attacker,
                // same with defender but opposite.
                // If we are in possession of the ball or it's free or opponent is in
                // possession, then we can attack if we are closest BUT we have to be in a situation where we are
                // allowed to attack, eg it can't be the other team's penalty or their kick off
                if (closest_to_ball_on_team && allowed_to_attack) {
                    log<INFO>("We are in the best position to attack, going for it!");
                    emit<Task>(std::make_unique<Attack>(ball_pos));
                    return;
                }

                // If we are in the best position to attack, but we can't because of the situation
                // eg because of a throw in for the opponent, then we should stick to a good spot and be ready to attack
                if (closest_to_ball_on_team && !allowed_to_attack) {
                    emit<Task>(std::make_unique<ReadyAttack>());
                    log<INFO>(
                        "We are in the best position to attack, but we can't because of the situation, so get ready to "
                        "attack");
                    return;
                }

                // If we can't attack, eg another robot is attacking, we don't want to get in the way.
                // We should hang back in the penalty box and wait in case the ball comes toward us.
                // We should only hang back if we are the furthest back, ignoring the goalie.
                // If there's no robots, assume we are alone and are the furthest back
                // Shouldn't happen, as that should make us the attacker
                bool furthest_back = robots ? utility::strategy::furthest_back(*robots, field.Hfw, sensors.Hrw) : true;
                if (furthest_back) {
                    log<INFO>("We are the furthest back, so we should defend");
                    emit<Task>(std::make_unique<Defend>());
                    return;
                }

                // If we're not the attacker, nor are we the robot hanging back to protect in case the opponent takes
                // the ball up towards our goal, we should help out the attacker however makes sense in the situation
                log<INFO>(
                    "We are not the attacker, nor are we the robot hanging back to protect, so we should help out");
                emit<Task>(std::make_unique<Support>());
            });

        // READY state
        on<Provide<FieldPlayerMsg>, When<Phase, std::equal_to, Phase::READY>>().then([this] {
            // todo Determine dynamically the best ready position
            Eigen::Isometry3d Hfr = Eigen::Isometry3d::Identity();
            emit<Task>(std::make_unique<WalkToFieldPosition>(Hfr, true));
        });
        // When not playing or in ready, stand still
        on<Provide<FieldPlayerMsg>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });
    }

}  // namespace module::purpose
