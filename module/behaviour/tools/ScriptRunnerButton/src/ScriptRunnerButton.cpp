#include "ScriptRunnerButton.h"

#include "extension/Configuration.h"
#include "extension/Script.h"

#include "message/behaviour/ServoCommand.h"
#include "message/platform/darwin/DarwinSensors.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"

namespace module {
namespace behaviour {
    namespace tools {

        using extension::Configuration;
        using extension::ExecuteScriptByName;

        using NUClear::message::CommandLineArguments;

        using message::platform::darwin::ButtonMiddleDown;
        using message::platform::darwin::DarwinSensors;

        using utility::behaviour::RegisterAction;
        using utility::behaviour::ActionPriorites;
        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;

        struct ExecuteNextScript {};

        void ScriptRunner::executeNextScript() {

            // If we have a script to execute
            if (!scripts.empty()) {

                // Get it and emit it
                auto script = scripts.front();
                emit(std::make_unique<ExecuteScriptByName>(id, script));
                scripts.pop();
            }
            // Otherwise we are done, shutdown
            else {
                powerplant.shutdown();
            }
        }

        ScriptRunnerButton::ScriptRunnerButton(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , sensorHandle()
            , scripts()
            , id(size_t(this) * size_t(this) - size_t(this)) {

            on<Configuration>,
                With<Trigger<CommandLineArguments>>("ScriptRunnerButton.yaml")
                    .then([this](const Configuration& config, const CommandLineArguments& args) {
                        // Use configuration here from file ScriptRunnerButon.yaml
                        Script_Delay = config["Script_Delay"].as<uint>();

                        NUClear::log<NUClear::INFO>("Executing: ", args.size() - 1, " scripts");

                        for (size_t i = 1; i < args.size(); ++i) {
                            NUClear::log<NUClear::INFO>("Queueing script ", args[i]);
                            scripts.push(args[i]);
                        }
                    });

            sensorHandle = on<Trigger<DarwinSensors>, Single>().then([this] {
                executeNextScript();
                sensorHandle.disable();
                sensorHandle.unbind();
            });

            emit<Scope::DIRECT>(std::make_unique<RegisterAction>(RegisterAction{
                id,
                "ScriptRunnerButton",
                {std::pair<float, std::set<LimbID>>(
                    1, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
                [this](const std::set<LimbID>&) {},
                [this](const std::set<LimbID>&) {
                    // We should always be the only running thing
                },
                [this](const std::set<ServoID>&)}));

            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
                RegisterAction{2,
                               "ScriptRunnerButton",
                               {std::pair<float, std::set<LimbID>>(
                                   100, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM})},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<ServoID>&) {}}));

            on<Trigger<ButtonMiddleDown>>().then([this] {
                std::this_thread::sleep_for(std::chrono::seconds(Script_Delay));

                emit(std::make_unique<ExecuteScriptByName>(2, std::vector<std::string>({"Nod.yaml"})));
            });
        }

    }  // namespace tools
}  // namespace behaviour
}  // namespace module
