#include "GetUpPlanner.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/planning/PlanGetUp.hpp"
#include "message/skill/GetUp.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::planning {

    using extension::Configuration;
    using message::input::Sensors;
    using message::planning::PlanGetUp;
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

        on<Provide<PlanGetUp>, Trigger<Sensors>>().then([this](const RunInfo& info, const Sensors& sensors) {
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

            // We have finished getting up so we can reset getting_up and decide if we want to get up again (we failed)
            // If we don't this will make our task lapse and other providers can take over
            if (getting_up && info.run_reason == RunInfo::SUBTASK_DONE) {
                getting_up = false;
            }

            // If we have been at recovery levels for long enough we can trigger a getup
            if (!getting_up && recovery_frames > cfg.count) {
                // Emit a getup
                emit<Task>(std::make_unique<GetUp>());
            }
            // We are still getting up
            else if (getting_up) {
                emit<Task>(std::make_unique<Idle>());
            }
        });

        // If we are interrupted make sure we know we are no longer getting up
        on<Stop<PlanGetUp>>().then([this] { getting_up = false; });
    }

}  // namespace module::planning
