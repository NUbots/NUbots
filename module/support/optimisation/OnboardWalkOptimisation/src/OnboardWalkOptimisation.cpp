#include "OnboardWalkOptimisation.hpp"

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/platform/webots/messages.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/OptimisationResetDone.hpp"
#include "message/support/optimisation/OptimisationTimeUpdate.hpp"

namespace module::support::optimisation {

    using extension::Configuration;
    using message::input::Sensors;
    using message::platform::ButtonLeftDown;
    using message::platform::ButtonMiddleDown;
    using message::platform::RawSensors;
    using message::platform::webots::OptimisationCommand;
    using message::support::optimisation::NSGA2Evaluating;
    using message::support::optimisation::OptimisationResetDone;
    using message::support::optimisation::OptimisationTimeUpdate;

    OnboardWalkOptimisation::OnboardWalkOptimisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("OnboardWalkOptimisation.yaml").then([this](const Configuration& config) {
            // Use configuration here from file OnboardWalkOptimisation.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.standing_angle = config["standing_angle"].as<float>();

            cfg.wait_time = config["wait_time"].as<float>();
        });

        on<Trigger<OptimisationCommand>>().then([this](const OptimisationCommand& msg) {
            if (msg.command == OptimisationCommand::CommandType::RESET_ROBOT) {
                NUClear::log<NUClear::DEBUG>("Onboard Resetting!");
                resetting = true;
            }
        });

        on<Trigger<Sensors>>().then("Stability Reset Check", [this](const Sensors& sensors) {
            if (resetting) {
                // Length of time from first standing upright
                const auto time_waited =
                    std::chrono::duration_cast<std::chrono::duration<float>>(NUClear::clock::now() - start_time)
                        .count();
                // Trigger the next individual after standing for the config value cfg.wait_time
                if (is_upright && (time_waited > cfg.wait_time)) {
                    NUClear::log<NUClear::DEBUG>(
                        fmt::format("Stood upright for {:.1f} seconds. Reset done.", time_waited));
                    resetting  = false;
                    is_upright = false;
                    emit(std::make_unique<OptimisationResetDone>());
                }

                // Transform to torso {t} from world {w} space
                Eigen::Matrix4d Hwt = sensors.Htw.inverse().matrix();
                // Basis Z vector of torso {t} in world {w} space
                Eigen::Vector3d uZTw = Hwt.block(0, 2, 3, 1);

                // Check if angle between torso z axis and world z axis is greater than config value cfg.fallen_angle
                if (!is_upright && std::acos(Eigen::Vector3d::UnitZ().dot(uZTw)) < cfg.standing_angle) {
                    is_upright = true;
                    // Start timer
                    start_time = NUClear::clock::now();
                    NUClear::log<NUClear::DEBUG>(fmt::format("Standing for {} seconds.", cfg.wait_time));
                }
                else if (is_upright && !(std::acos(Eigen::Vector3d::UnitZ().dot(uZTw)) < cfg.standing_angle)) {
                    is_upright = false;
                    NUClear::log<NUClear::DEBUG>("No longer standing.");
                }
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
        on<Trigger<ButtonMiddleDown>, Single>().then([this] {});
    }
}  // namespace module::support::optimisation
