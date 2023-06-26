#include "WalkInsideBoundedBox.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/GameState.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/strategy/WalkInsideBoundedBox.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::strategy {

    using extension::Configuration;
    using WalkInsideBoundedBoxTask = message::strategy::WalkInsideBoundedBox;
    using utility::support::Expression;
    using Ball = message::localisation::Ball;
    using message::localisation::Field;
    using message::strategy::WalkToFieldPosition;

    WalkInsideBoundedBox::WalkInsideBoundedBox(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {
        on<Configuration>("WalkInsideBoundedBox.yaml").then([this](const Configuration& config) {
            this->log_level          = config["log_level"].as<NUClear::LogLevel>();
            cfg.bounded_region_x_min = config["bounded_region_x_min"].as<Expression>();
            cfg.bounded_region_x_max = config["bounded_region_x_max"].as<Expression>();
            cfg.bounded_region_y_min = config["bounded_region_y_min"].as<Expression>();
            cfg.bounded_region_y_max = config["bounded_region_y_max"].as<Expression>();
        });
        on<Provide<WalkInsideBoundedBoxTask>, Trigger<Ball>, With<Field>>().then(
            [this](const WalkInsideBoundedBoxTask& walk_inside_bounded_box_task, const Ball& ball, const Field& field) {
                // Get the current position of the ball on the field
                Eigen::Isometry3d Hfw = field.Hfw;
                Eigen::Vector3d rBFf  = Hfw * ball.rBWw;

                // Desired position of robot on field
                Eigen::Vector3d rDFf = Eigen::Vector3d::Zero();

                // Check if the ball is in the bounding box
                if (rBFf.x() > cfg.bounded_region_x_min && rBFf.x() < cfg.bounded_region_x_max
                    && rBFf.y() > cfg.bounded_region_y_max && rBFf.y() < cfg.bounded_region_y_min) {
                    // Do nothing as ball is inside of defending region, play normally
                    log<NUClear::DEBUG>("Ball is inside of bounding box");
                }
                else {
                    // If ball is in a region parallel and outside own bounding box of robot we clamp in the y
                    // direction and move to 1m behind ball
                    if (rBFf.x() >= 0 && rBFf.y() > cfg.bounded_region_y_min) {
                        log<NUClear::DEBUG>("Ball is in own half and outside bounding box");
                        // Clamp desired position to bounding box and try stay 1m behind ball
                        rDFf.x() = std::clamp(rBFf.x() + 1.0, cfg.bounded_region_x_min, cfg.bounded_region_x_max);
                        rDFf.y() = std::clamp(rBFf.y(), cfg.bounded_region_y_max, cfg.bounded_region_y_min);
                    }
                    else {
                        log<NUClear::DEBUG>("Ball is in opponents half and outside bounding box");
                        // Clamp desired position to bounding box
                        rDFf.x() = std::clamp(rBFf.x(), cfg.bounded_region_x_min, cfg.bounded_region_x_max);
                        rDFf.y() = std::clamp(rBFf.y(), cfg.bounded_region_y_max, cfg.bounded_region_y_min);
                    }

                    // Emit task to walk to desired position with heading facing opponents side of field
                    emit<Task>(std::make_unique<WalkToFieldPosition>(Eigen::Vector3f(rDFf.x(), rDFf.y(), 0), -M_PI));
                }
            });
    }

}  // namespace module::strategy
