#include "AlignBallToGoal.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/FilteredBall.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/strategy/AlignBallToGoal.hpp"
#include "message/support/FieldDescription.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::localisation::FilteredBall;
    using message::planning::TurnAroundBall;
    using message::support::FieldDescription;
    using AlignBallToGoalTask = message::strategy::AlignBallToGoal;
    using utility::support::Expression;

    AlignBallToGoal::AlignBallToGoal(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("AlignBallToGoal.yaml").then([this](const Configuration& config) {
            // Use configuration here from file AlignBallToGoal.yaml
            this->log_level             = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_distance_threshold = config["ball_distance_threshold"].as<float>();
            cfg.angle_threshold         = config["angle_threshold"].as<Expression>();
        });

        on<Provide<AlignBallToGoalTask>,
           With<FilteredBall>,
           With<Field>,
           With<Sensors>,
           With<FieldDescription>,
           Every<30, Per<std::chrono::seconds>>>()
            .then([this](const FilteredBall& ball,
                         const Field& field,
                         const Sensors& sensors,
                         const FieldDescription& field_description) {
                // If the ball is close, align towards the goal
                float distance_to_ball = ball.rBRr.head(2).norm();
                if (distance_to_ball < cfg.ball_distance_threshold) {
                    // Get the robot's position (pose) on the field
                    Eigen::Isometry3f Hrf = Eigen::Isometry3f(sensors.Hrw) * Eigen::Isometry3f(field.Hfw.inverse());

                    // Goal position relative to robot
                    Eigen::Vector3f rGFf =
                        Eigen::Vector3f(-field_description.dimensions.field_length * 0.5f, 0.0f, 0.0f);
                    Eigen::Vector3f rGRr = Hrf * rGFf;

                    // Find the angle to the goal - should be as close as possible to 0 to be aligned
                    float kick_angle = std::atan2(rGRr.y(), rGRr.x());

                    // Only align if we are not within a threshold of the goal
                    if (std::fabs(kick_angle) > cfg.angle_threshold) {
                        if (kick_angle < 0.0f) {
                            emit<Task>(std::make_unique<TurnAroundBall>(true));
                        }
                        else {
                            emit<Task>(std::make_unique<TurnAroundBall>(false));
                        }
                    }
                }
            });
    }

}  // namespace module::strategy
