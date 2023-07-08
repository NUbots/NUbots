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
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.count     = config["count"].as<int>();
            cfg.cos_angle = config["angle"].as<Expression>();  // Take the cosine for easier comparision later
            cfg.gyro      = config["gyro"].as<Expression>();
            cfg.g         = config["g"].as<Expression>();
            cfg.acc       = config["acc"].as<Expression>();
        });

        on<Provide<GetUpWhenFallen>, Uses<GetUp>, Trigger<Sensors>>().then(
            [this](const Uses<GetUp>& getup, const Sensors& sensors) {
                if (getup.run_state == GroupInfo::RunState::RUNNING && !getup.done) {
                    emit<Task>(std::make_unique<Idle>());
                    log<NUClear::DEBUG>("Executing getup");
                    return;
                }
                // Transform to torso{t} from world{w} space
                Eigen::Matrix4d Hwt = sensors.Htw.inverse().matrix();
                // Basis Z vector of torso {t} in world {w} space
                Eigen::Vector3d uZTw = Hwt.block(0, 2, 3, 1);

                double angle = std::acos(Eigen::Vector3d::UnitZ().dot(uZTw));
                log<NUClear::DEBUG>("Angle: ", angle);

                // // Check if angle between torso z axis and world z axis is greater than config value
                if (angle > cfg.cos_angle) {
                    emit<Task>(std::make_unique<GetUp>());
                    log<NUClear::DEBUG>("Execute getup");
                }
                // Otherwise do not need to get up so emit no tasks
            });
    }

}  // namespace module::planning
