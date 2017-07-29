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

        using message::platform::darwin::ButtonMiddleDown;
        using message::platform::darwin::ButtonLeftDown;

        using utility::behaviour::RegisterAction;
        using utility::behaviour::ActionPriorites;
        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;

        ScriptRunnerButton::ScriptRunnerButton(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)) {

            on<Configuration>("ScriptRunnerButton.yaml").then([this](const Configuration& config) {
                // Use configuration here from file ScriptRunnerButton.yaml
            });

            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
                RegisterAction{2,
                               "Jump",
                               {std::pair<float, std::set<LimbID>>(
                                   100, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM})},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<ServoID>&) {}}));

            on<Trigger<ButtonMiddleDown>>().then([this] {
                std::this_thread::sleep_for(std::chrono::seconds(2));

                emit(std::make_unique<ExecuteScriptByName>(2, std::vector<std::string>({"Jump.yaml"})));
            });
        }

    }  // namespace tools
}  // namespace behaviour
}  // namespace module
