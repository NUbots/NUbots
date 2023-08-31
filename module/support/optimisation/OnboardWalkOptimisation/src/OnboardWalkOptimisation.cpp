#include "OnboardWalkOptimisation.hpp"

#include "extension/Configuration.hpp"
#include "message/support/optimisation/OptimisationResetDone.hpp"
#include "message/support/optimisation/OptimisationTimeUpdate.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/input/Sensors.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"

namespace module::support::optimisation {

using extension::Configuration;
    using message::support::optimisation::OptimisationResetDone;
    using message::support::optimisation::OptimisationTimeUpdate;
    using message::support::optimisation::NSGA2Evaluating;
    using message::platform::RawSensors;
    using message::platform::webots::OptimisationCommand;
    using message::input::Sensors;
    using message::platform::ButtonMiddleDown;
    using message::platform::ButtonLeftDown;

OnboardWalkOptimisation::OnboardWalkOptimisation(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("OnboardWalkOptimisation.yaml").then([this](const Configuration& config) {
        // Use configuration here from file OnboardWalkOptimisation.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();

        cfg.standing_angle = config["standing_angle"].as<float>();

        cfg.wait_time = config["wait_time"].as<float>();
    });

    on<Trigger<OptimisationCommand>>().then([this](const OptimisationCommand& msg) {
        if (msg.command == OptimisationCommand::CommandType::RESET_WORLD) {
            NUClear::log<NUClear::DEBUG>("Onboard Resetting!");
            resetting = true;
        }
    });

    on<Trigger<Sensors>>().then("Stability Reset Check", [this](const Sensors& sensors) {
        // Length of time from first standing upright
        const auto time_waited =
            std::chrono::duration_cast<std::chrono::duration<float>>(NUClear::clock::now() - start_time).count();
        // Trigger the next individual after standing for the config value cfg.wait_time
        if (resetting && is_upright && (time_waited > cfg.wait_time)) {
            NUClear::log<NUClear::DEBUG>("Stood upright for {} seconds. Reset started.", cfg.wait_time);
            resetting = false;
            emit(std::make_unique<OptimisationResetDone>());
        }

        // Transform to torso {t} from world {w} space
        Eigen::Matrix4d Hwt = sensors.Htw.inverse().matrix();
        // Basis Z vector of torso {t} in world {w} space
        Eigen::Vector3d uZTw = Hwt.block(0, 2, 3, 1);

        // Check if angle between torso z axis and world z axis is greater than config value cfg.fallen_angle
        if (resetting && !is_upright && std::acos(Eigen::Vector3d::UnitZ().dot(uZTw)) < cfg.standing_angle) {
            is_upright = true;
            // Start timer
            start_time = NUClear::clock::now();
            NUClear::log<NUClear::DEBUG>("Starting to stand upright at time", start_time.time_since_epoch());
        }
        else if (resetting) {
            is_upright = false;
        }
    });

    // Reset for next run or terminate when paused
    on<Trigger<ButtonLeftDown>, Single>().then([this] {
        NUClear::log<NUClear::DEBUG>("Left Button Pressed");
        if (false) {
            emit(std::make_unique<OptimisationResetDone>());
        }
    });

    // Pause optimisation
    on<Trigger<ButtonMiddleDown>, Single>().then([this] {
    });
}
}  // namespace module::support::optimisation
