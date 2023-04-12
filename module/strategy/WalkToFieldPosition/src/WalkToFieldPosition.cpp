#include "WalkToFieldPosition.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::planning::WalkTo;
    using WalkToFieldPositionTask = message::strategy::WalkToFieldPosition;
    using message::input::Sensors;
    using message::localisation::Field;

    WalkToFieldPosition::WalkToFieldPosition(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("WalkToFieldPosition.yaml").then([this](const Configuration& config) {
            // Use configuration here from file WalkToFieldPosition.yaml
            this->log_level  = config["log_level"].as<NUClear::LogLevel>();
            cfg.align_radius = config["align_radius"].as<float>();
        });

        on<Provide<WalkToFieldPositionTask>, With<Field>, With<Sensors>, Every<30, Per<std::chrono::seconds>>>().then(
            [this](const WalkToFieldPositionTask& walk_to_field_position, const Field& field, const Sensors& sensors) {
                // Transform the desired field {f} position into ground {g} space
                const Eigen::Isometry3f Hfw = Eigen::Isometry3f(field.Hfw.cast<float>());
                const Eigen::Isometry3f Hgw = Eigen::Isometry3f(sensors.Hgw.cast<float>());
                const Eigen::Isometry3f Hgf = Hgw * Hfw.inverse();
                const Eigen::Vector3f rPFf(walk_to_field_position.rPFf.x(), walk_to_field_position.rPFf.y(), 0.0f);
                const Eigen::Vector3f rPGg(Hgf * rPFf);

                // If we are close to the field position, align with the desired heading
                float heading;
                if (rPGg.head(2).norm() < cfg.align_radius) {
                    // Create a unit vector in the direction of the desired heading in field space
                    const Eigen::Vector3f uHFf(std::cos(walk_to_field_position.heading),
                                               std::sin(walk_to_field_position.heading),
                                               0.0f);
                    // Rotate the desired heading in field space to ground space
                    const Eigen::Vector3f uHGg(Hgf.linear() * uHFf);
                    heading = std::atan2(uHGg.y(), uHGg.x());
                }
                // Otherwise, walk directly to the field position
                else {
                    heading = std::atan2(rPGg.y(), rPGg.x());
                }

                // Emit a task to walk to the field position
                auto walk_to_point     = std::make_unique<WalkTo>();
                walk_to_point->rPGg    = rPGg;
                walk_to_point->heading = heading;
                emit<Task>(walk_to_point);
            });
    }

}  // namespace module::strategy
