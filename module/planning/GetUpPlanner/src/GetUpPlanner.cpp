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
            cfg.cos_angle = std::cos(config["angle"].as<Expression>());  // Take the cosine for easier comparision later
            cfg.gyro      = config["gyro"].as<Expression>();
            cfg.g         = config["g"].as<Expression>();
            cfg.acc       = config["acc"].as<Expression>();
        });

        on<Provide<GetUpWhenFallen>, Uses<GetUp>, Trigger<Sensors>>().then(
            [this](const RunInfo& info, const Uses<GetUp>& getup, const Sensors& sensors) {
                // Other trigger means we got a new sensors object
                if (info.run_reason == RunInfo::OTHER_TRIGGER) {
                    // Calculate our recovery values
                    // Htw(2, 2) contains the dot product of the z axis of the torso with the world z axis (cos_angle)
                    double cos_angle = sensors.Htw(2, 2);
                    double acc       = sensors.accelerometer.norm() - cfg.g;
                    double gyro      = sensors.gyroscope.x() + sensors.gyroscope.y() + sensors.gyroscope.z();

                    // Check if we are at recovery levels
                    bool recovery = cos_angle < cfg.cos_angle && acc < cfg.acc && gyro < cfg.gyro;

                    // Accumulate recovery frames or reset if we are not at recovery levels
                    recovery_frames = recovery ? recovery_frames + 1 : 0;
                }

                bool running = getup.run_state == GroupInfo::RunState::RUNNING;
                bool queued  = getup.run_state == GroupInfo::RunState::QUEUED;

                // If we have been at recovery levels for long enough we can trigger a getup, but only if we haven't
                // already requested one
                if (recovery_frames > cfg.count && getup.run_state == GroupInfo::RunState::NO_TASK) {
                    emit<Task>(std::make_unique<GetUp>());
                }
                // Keep requesting the get up if running and not done, or if we are queued and we need to get up
                else if ((recovery_frames > cfg.count && queued) || (running && !getup.done)) {
                    emit<Task>(std::make_unique<Idle>());
                }
                // Otherwise do not need to get up so emit no tasks
            });
    }

}  // namespace module::planning
