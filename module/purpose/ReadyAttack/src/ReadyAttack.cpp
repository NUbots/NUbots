#include "ReadyAttack.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/purpose/Player.hpp"
#include "message/strategy/FindBall.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/math/euler.hpp"

namespace module::purpose {

    using extension::Configuration;

    using message::input::GameState;
    using message::localisation::Ball;
    using message::localisation::Field;
    using ReadyAttackTask = message::purpose::ReadyAttack;
    using message::strategy::FindBall;
    using message::strategy::PositionBehindBall;
    using message::strategy::WalkToFieldPosition;
    using message::support::FieldDescription;

    using utility::math::euler::pos_rpy_to_transform;

    ReadyAttack::ReadyAttack(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("ReadyAttack.yaml").then([this](const Configuration& config) {
            // Use configuration here from file ReadyAttack.yaml
            this->log_level             = config["log_level"].as<NUClear::LogLevel>();
            cfg.center_circle_offset    = config["center_circle_offset"].as<double>();
            cfg.penalty_defend_distance = config["penalty_defend_distance"].as<double>();
        });

        on<Provide<ReadyAttackTask>, With<GameState>, With<FieldDescription>, With<Field>, Optional<With<Ball>>>().then(
            [this](const GameState& game_state,
                   const FieldDescription& fd,
                   const Field& field,
                   const std::shared_ptr<const Ball>& ball) {
                // If there's no ball, find it
                // This shouldn't happen, as it should be taken care of higher up
                if (!ball) {
                    log<INFO>("No ball, finding it...");
                    emit<Task>(std::make_unique<FindBall>());
                    return;
                }

                // Ready state may happen during penalty positioning or kick off
                // Kickoff will happen in normal mode
                if (game_state.mode == GameState::Mode::NORMAL) {
                    log<INFO>("Waiting for kick off...");
                    // Waiting for kick off, position outside the center circle
                    Eigen::Vector3d rPFf =
                        Eigen::Vector3d(0, 0, fd.dimensions.center_circle_diameter / 2 + cfg.center_circle_offset);
                    emit<Task>(std::make_unique<WalkToFieldPosition>(
                        utility::math::euler::pos_rpy_to_transform(rPFf, Eigen::Vector3d(0, 0, -M_PI)),
                        true));
                    return;
                }

                // If we are not waiting for kick off, it is the penalty positioning phase
                // Determine if we are the attacker or not
                bool attacker = game_state.secondary_state.team_performing == game_state.team.team_id;

                // If we are defending, position between the ball and our goal at the distance specified in the rules
                if (!attacker) {
                    log<INFO>("Defending penalty, positioning...");
                    // Position of the center of the goals in field coordinates
                    Eigen::Vector3d rGFf =
                        Eigen::Vector3d((fd.dimensions.field_length / 2) + fd.dimensions.goal_depth, 0.0, 0.0);
                    // Position of the ball in field coordinates
                    Eigen::Vector3d rBFf = field.Hfw * ball->rBWw;

                    // Unit vector from goal to ball
                    Eigen::Vector3d uGBf = (rGFf - rBFf).normalized();
                    // Move the ball position by the penalty defend distance
                    Eigen::Vector3d rPFf = rBFf + (uGBf * cfg.penalty_defend_distance);

                    // Rotation should face the ball, get the angle from the field to the ball
                    double angle = std::atan2(rBFf.y(), rBFf.x());

                    // Emit a task to walk to the position, facing the ball
                    emit<Task>(std::make_unique<WalkToFieldPosition>(
                        utility::math::euler::pos_rpy_to_transform(rPFf, Eigen::Vector3d(0, 0, angle)),
                        true));
                }
                log<INFO>("Attacking penalty, positioning...");
                // We are attacking, and should position to take the ball towards the opponent's goal
                emit<Task>(std::make_unique<PositionBehindBall>());
            });
    }

}  // namespace module::purpose
