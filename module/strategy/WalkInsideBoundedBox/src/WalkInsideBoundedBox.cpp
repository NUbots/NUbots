#include "WalkInsideBoundedBox.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/FilteredBall.hpp"
#include "message/planning/KickTo.hpp"
#include "message/strategy/AlignBallToGoal.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkInsideBoundedBox.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::strategy {

    using extension::Configuration;
    using DefendTask = message::strategy::WalkInsideBoundedBox;
    using utility::support::Expression;
    using Ball = message::localisation::FilteredBall;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::strategy::AlignBallToGoal;
    using message::strategy::StandStill;
    using message::strategy::WalkToFieldPosition;

    WalkInsideBoundedBox::WalkInsideBoundedBox(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Defend.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Defend.yaml
            this->log_level      = config["log_level"].as<NUClear::LogLevel>();
            cfg.defending_region = Eigen::Vector4f(config["defending_region"].as<Expression>());

            log<NUClear::DEBUG>("cfg.defending_region ", cfg.defending_region.transpose());
        });

        on<Provide<DefendTask>, Trigger<Ball>, With<Field>, With<Sensors>>().then(
            [this](const DefendTask& defend_task, const Ball& ball, const Field& field, const Sensors& sensor) {
                Eigen::Isometry3f Hfw = Eigen::Isometry3f(field.Hfw.cast<float>());
                Eigen::Isometry3f Hrw = Eigen::Isometry3f(sensor.Hrw.cast<float>());
                Eigen::Isometry3f Hfr = Hfw * Hrw.inverse();
                Eigen::Vector3f rBFf  = Hfw * ball.rBWw;
                Eigen::Vector3f rRFf  = Hfr.translation();

                // The distance of the robot from the ball
                double robot_distance_to_ball =
                    sqrt(abs(pow(rBFf.x() - rRFf.x(), 2.0) - pow(rBFf.y() - rRFf.y(), 2.0)));

                log<NUClear::DEBUG>("rRFf: ", rRFf.transpose());
                log<NUClear::DEBUG>("rBFf: ", rBFf.transpose());
                log<NUClear::DEBUG>("Dist: ", robot_distance_to_ball);

                // Check if the ball is in the defending region
                if (rBFf.x() > cfg.defending_region(0) && rBFf.x() < cfg.defending_region(1)
                    && rBFf.y() > cfg.defending_region(2) && rBFf.y() < cfg.defending_region(3)) {

                    log<NUClear::DEBUG>("Ball inside of defending region");

                    // Do nothing, play normally
                }
                else {  // Determines robots as ball has left bounded region
                    // Robot - Defending position on field
                    Eigen::Vector3f rDFf = Eigen::Vector3f::Zero();

                    log<NUClear::DEBUG>("rDFf x: ", rDFf.x());
                    log<NUClear::DEBUG>("rDFf y: ", rDFf.y());

                    // If ball is in own half and outside the defending bounding box of robot we clamp in the y
                    // direction and move to 1m behind ball
                    if (rBFf.x() >= 0 && rBFf.y() > cfg.defending_region(3)) {
                        log<NUClear::DEBUG>("Ball is in own half and in other region");

                        // Calculate the defender position
                        // Clamps to x direction of the ball and bounding box an 1 metre behind the ball
                        rDFf.x() = std::clamp(rBFf.x(), cfg.defending_region(0), cfg.defending_region(1));
                        rDFf.y() = std::clamp(rBFf.y(), cfg.defending_region(2), cfg.defending_region(3));
                        rDFf.x() += 1.0;  // For positioning of robot 1 metre behind ball when ball is in own half but
                                          // not inside robots bounded box
                    }
                    else {
                        // Calculate the defender position
                        // Robot clamped to defending bounding box
                        rDFf.x() = std::clamp(rBFf.x(), cfg.defending_region(0), cfg.defending_region(1));
                        rDFf.y() = std::clamp(rBFf.y(), cfg.defending_region(2), cfg.defending_region(3));
                    }

                    // Walk to determined position given by vector rDFf
                    emit<Task>(std::make_unique<WalkToFieldPosition>(Eigen::Vector3f(rDFf.x(), rDFf.y(), 0), -M_PI));
                    log<NUClear::DEBUG>("Ball is outside of defending region");
                }
            });
    }

}  // namespace module::strategy
