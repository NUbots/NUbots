#include "GetUpPlanner.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/planning/GetUpWhenFallen.hpp"
#include "message/skill/GetUp.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::planning {

    using extension::Configuration;
    using message::input::Sensors;
    using message::planning::GetUpWhenFallen;
    using message::skill::GetUp;
    using utility::support::Expression;

    GetUpPlanner::GetUpPlanner(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("GetUpPlanner.yaml").then([this](const Configuration& config) {
            this->log_level  = config["log_level"].as<NUClear::LogLevel>();
            cfg.fallen_angle = config["fallen_angle"].as<float>();
        });

        on<Provide<GetUpWhenFallen>, Uses<GetUp>, Trigger<Sensors>>().then(
            [this](const RunInfo& info, const Uses<GetUp>& getup, const Sensors& sensors) {
                if (getup.run_state == GroupInfo::RunState::RUNNING && !getup.done) {
                    emit<Task>(std::make_unique<Idle>());
                    log<NUClear::DEBUG>("Idle");
                    return;
                }
                // Transform to torso{t} from world{w} space
                Eigen::Matrix4d Hwt = sensors.Htw.inverse().matrix();
                // Basis Z vector of torso {t} in world {w} space
                Eigen::Vector3d uZTw = Hwt.block(0, 2, 3, 1);

                // Get the angle of the robot with the world z axis
                double angle = std::acos(Eigen::Vector3d::UnitZ().dot(uZTw));
                log<NUClear::DEBUG>("Angle: ", angle);

                // // Check if angle between torso z axis and world z axis is greater than config value
                if (angle > cfg.fallen_angle) {
                    emit<Task>(std::make_unique<GetUp>());
                    log<NUClear::DEBUG>("Execute getup");
                }
                // Otherwise do not need to get up so emit no tasks
            });
    }

}  // namespace module::planning
