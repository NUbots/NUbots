#include "FieldPlayer.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/localisation/Robot.hpp"
#include "message/purpose/FieldPlayer.hpp"
#include "message/strategy/Who.hpp"

#include "utility/strategy/soccer_strategy.hpp"

namespace module {

    using extension::Configuration;
    using message::localisation::Robots;
    using FieldPlayerMsg = message::purpose::FieldPlayer;
    using message::strategy::Who;


    FieldPlayer::FieldPlayer(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("FieldPlayer.yaml").then([this](const Configuration& config) {
            // Use configuration here from file FieldPlayer.yaml
            this->log_level    = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_threshold = config["ball_threshold"].as<double>();
        });

        // When not playing or in ready, stand still
        on<Provide<FieldPlayerMsg>>().then([this] { emit<Task>(std::make_unique<StandStill>()); });

        // READY state
        on<Provide<FieldPlayerMsg>, When<Phase, std::equal_to, Phase::READY>>().then([this] {
            // todo Determine dynamically the best ready position
            Hfr = Eigen::Isometry3d::Identity();
            emit<Task>(std::make_unique<WalkToFieldPosition>(Hfr, true));
        });

        // PLAYING state
        on<Provide<FieldPlayerMsg>,
           With<Ball>,
           With<Robots>,
           With<Sensors>,
           With<Field>,
           When<Phase, std::equal_to, Phase::PLAYING>>()
            .then([this](const Ball& ball, const Robots& robots, const Sensors& sensors, const Field& field) {
                log<INFO>("Playing as a FieldPlayer");

                // If the robot is unsure what is happening, it should find out before playing
                // if (state == UNKNOWN) {
                //    log<DEBUG>("Not enough info, checking");
                //    emit<Task>(std::make_unique<LookAround>());
                //    emit<Task>(std::make_unique<TurnOnSpot>());
                //    return;
                // }
                // Else do something

                // Determine who has the ball
                ball_pos =
                    utility::strategy::ball_possession(ball.rBWw, robots, field.Hfw, field.Hrw, cfg.ball_threshold);

                // True if we are closest to the ball on our team
                closest_to_ball_on_team =
                    utility::strategy::closest_to_ball_on_team(ball.rBWw, robots, field.Hfw, field.Hrw);

                // todo Check if we can attack
                allowed_to_attack = true;

                // Todo: if another robot/s within a threshold distance, the lowest robot id should be the attacker,
                // same with defender but opposite If we are in possession of the ball, Or it's free or opponent is in
                // possession, then we can attack if we are closest BUT we have to be in a situation where we are
                // allowed to attack, eg it can't be the other team's penalty or their kick off
                if (ball_pos == Who::SELF || (ball_pos != TEAMMATE && closest_to_ball_on_team) and allowed_to_attack) {
                    emit<Task>(std::make_unique<Attack>(ball_pos));
                }
                // If we are in the best position to attack, but we can't because of the situation
                // eg because of a throw in for the opponent, then we should stick to a good spot and be ready to attack
                else if (ball_possession == SELF && closest_to_ball && !allowed_to_attack) {
                    emit<Task>(std::make_unique<ReadyAttack>());
                }

                // If we can't attack, eg another robot is attacking, we don't want to get in the way
                // We should hang back in the penalty box and wait in case the ball comes toward us.
                // We should only hang back if we are the furthest back
                furthest_back = utility::strategy::furthest_back(robots, field.Hfw, field.Hrw);
                else if (furthest_back) {
                    emit<Task>(std::make_unique<Defend>());
                }

                // If we're not the attacker, nor are we the robot hanging back to protect in case the opponent takes
                // the ball up towards our goal, we should help out the attacker however makes sense in the situation
                else {
                    emit<Task>(std::make_unique<Support>());
                }
            });
    }

}  // namespace module
