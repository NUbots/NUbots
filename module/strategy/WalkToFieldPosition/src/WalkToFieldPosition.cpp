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
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Provide<WalkToFieldPositionTask>, With<Field>, With<Sensors>, Every<30, Per<std::chrono::seconds>>>().then(
            [this](const WalkToFieldPositionTask& walk_to_field_position, const Field& field, const Sensors& sensors) {
                // Transform the field position into the robot's torso frame
                const Eigen::Isometry3f Hfw = Eigen::Isometry3d(field.Hfw).cast<float>();
                const Eigen::Isometry3f Htw = Eigen::Isometry3f(sensors.Htw.cast<float>());
                const Eigen::Isometry3f Htf = Htw * Hfw.inverse();
                log<NUClear::DEBUG>("rWFf :", Hfw.translation().transpose());

                const Eigen::Vector3f rPFf(walk_to_field_position.rPFf.x(), walk_to_field_position.rPFf.y(), 0.0f);
                const Eigen::Vector3f rPTt(Htf * rPFf);

                // Emit a task to walk to the field position
                // emit<Task>(std::make_unique<WalkTo>(rPTt));
            });
    }

}  // namespace module::strategy
