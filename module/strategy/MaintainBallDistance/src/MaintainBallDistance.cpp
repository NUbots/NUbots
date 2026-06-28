#include "MaintainBallDistance.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/strategy/MaintainBallDistance.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/euler.hpp"

namespace module::strategy {

    using extension::Configuration;

    using MaintainBallDistanceTask = message::strategy::MaintainBallDistance;

    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::strategy::WalkToFieldPosition;

    using utility::math::angle::vector_to_bearing;
    using utility::math::euler::pos_rpy_to_transform;

    MaintainBallDistance::MaintainBallDistance(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("MaintainBallDistance.yaml").then([this](const Configuration& config) {
            // Use configuration here from file MaintainBallDistance.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<MaintainBallDistanceTask>, With<Ball>, With<Field>, With<Sensors>>().then([this](const MaintainBallDistanceTask& task, const Ball& ball, const Field& field, const Sensors& sensors) {
            // Ball position in field frame
            const Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;

            // Robot position in field frame
            const Eigen::Isometry3d Hfr = field.Hfw * sensors.Hrw.inverse();
            const Eigen::Vector3d rRFf  = Hfr.translation();

            // Vector from ball to robot in field frame
            const Eigen::Vector3d rRBf      = rRFf - rBFf;
            const double current_distance   = rRBf.head<2>().norm();

            // If already far enough, let the parent task drive
            if (current_distance >= task.min_distance) {
                log<DEBUG>("Already at safe distance from ball.");
                return;
            }

            // Unit vector from ball toward robot
            // Degenerate case: robot is too close to ball, fall back to +x field axis
            Eigen::Vector3d uRBf;
            if (current_distance < 1e-3) {
                log<DEBUG>("Robot is too close to ball, backing away along +x field axis.");
                uRBf = Eigen::Vector3d(1.0, 0.0, 0.0);
            }
            else {
                uRBf = rRBf / current_distance;
            }

            // Target is min_distance from the ball along the ball-to-robot axis
            const Eigen::Vector3d target = rBFf + uRBf * task.min_distance;

            log<DEBUG>("Backing away from ball: current distance ", current_distance, "m, target ", task.min_distance, "m.");

            // Face the ball while backing up
            const double heading = vector_to_bearing((rBFf - target).head<2>());

            emit<Task>(std::make_unique<WalkToFieldPosition>(
                pos_rpy_to_transform(target, Eigen::Vector3d(0, 0, heading)),
                true));
        });
    }

}  // namespace module::strategy
