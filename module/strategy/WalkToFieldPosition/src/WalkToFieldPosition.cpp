#include "WalkToFieldPosition.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::planning::WalkTo;
    using message::strategy::StandStill;
    using WalkToFieldPositionTask = message::strategy::WalkToFieldPosition;

    WalkToFieldPosition::WalkToFieldPosition(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("WalkToFieldPosition.yaml").then([this](const Configuration& config) {
            // Use configuration here from file WalkToFieldPosition.yaml
            this->log_level    = config["log_level"].as<NUClear::LogLevel>();
            cfg.align_radius   = config["align_radius"].as<float>();
            cfg.stop_tolerance = config["stop_tolerance"].as<float>();
        });

        on<Provide<WalkToFieldPositionTask>, With<Field>, With<Sensors>, Every<30, Per<std::chrono::seconds>>>().then(
            [this](const WalkToFieldPositionTask& walk_to_field_position, const Field& field, const Sensors& sensors) {
                const Eigen::Isometry3f Hfw = Eigen::Isometry3f(field.Hfw.cast<float>());
                const Eigen::Isometry3f Hrw = Eigen::Isometry3f(sensors.Hrw.cast<float>());
                const Eigen::Isometry3f Hrf = Hrw * Hfw.inverse();
                const Eigen::Isometry3f Hfr = Hrf.inverse();
                const Eigen::Vector3f rPFf(walk_to_field_position.rPFf.x(), walk_to_field_position.rPFf.y(), 0.0f);

                // Create a unit vector in the direction of the desired heading in field space
                const Eigen::Vector3f uHFf(std::cos(walk_to_field_position.heading),
                                           std::sin(walk_to_field_position.heading),
                                           0.0f);

                // Transform the field position from field {f} space to robot {r} space
                const Eigen::Vector3f rPRr(Hrf * rPFf);

                // Compute the current position error and heading error in field {f} space
                const float position_error = (Hfr.translation().head(2) - rPFf.head(2)).norm();
                const Eigen::Vector3f uXRf = Hfr.linear().col(0).head(2);
                const float heading_error  = std::acos(std::max(-1.0f, std::min(1.0f, uXRf.dot(uHFf.head(2)))));
                log<NUClear::DEBUG>("Position error: ", position_error, " Heading error: ", heading_error);

                // If the error in the desired field position and heading is low enough, stand still
                if (position_error < cfg.stop_tolerance && heading_error < cfg.stop_tolerance) {
                    emit<Task>(std::make_unique<StandStill>());
                }
                // If we are getting close to the field position begin to align with the desired heading in field space
                else if (position_error < cfg.align_radius) {
                    // Rotate the desired heading in field {f} space to robot space
                    const Eigen::Vector3f uHRr(Hrf.linear() * uHFf);
                    const float desired_heading = std::atan2(uHRr.y(), uHRr.x());
                    emit<Task>(std::make_unique<WalkTo>(rPRr, desired_heading));
                }
                // Otherwise, walk directly to the field position
                else {
                    const float desired_heading = std::atan2(rPRr.y(), rPRr.x());
                    emit<Task>(std::make_unique<WalkTo>(rPRr, desired_heading));
                }
            });
    }

}  // namespace module::strategy
