#include "ScriptRunner.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/skill/Script.hpp"

namespace module::purpose {

    using extension::Configuration;
    using extension::behaviour::Task;

    using message::actuation::LimbsSequence;
    using message::platform::ButtonMiddleDown;
    using NUClear::message::CommandLineArguments;

    using utility::skill::load_script;

    ScriptRunner::ScriptRunner(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        // Get the scripts to run from the command line
        on<Configuration, With<CommandLineArguments>>("ScriptRunner.yaml")
            .then([this](const Configuration& config, const CommandLineArguments& args) {
                scripts = config["scripts"].as<std::vector<std::string>>();

                // Check for scripts entered in the command line
                if (args.size() > 1) {
                    log<NUClear::INFO>("Executing: ", args.size() - 1, " script from argument");
                    scripts.clear();
                    std::copy(std::next(args.begin(), 1), args.end(), std::back_inserter(scripts));
                }

                // If scripts are in the config file
                else if (!scripts.empty()) {
                    log<NUClear::INFO>("Executing: ", scripts.size(), " script from config");
                }

                // No default scripts or command line scripts
                else {
                    log<NUClear::WARN>("No scripts loaded");
                }
            });

        on<Trigger<ButtonMiddleDown>, Single>().then([this] {
            log<NUClear::INFO>("Middle button pressed, running scripts: ");
            for (auto& script : scripts) {
                log<NUClear::INFO>("\n", script);
            }
            emit<Task>(load_script<LimbsSequence>(scripts));
        });
    }
}  // namespace module::purpose
