#include "PythonScript.h"

#include "extension/Configuration.h"

#include "message/behaviour/PythonScript.h"
#include "message/extension/Script.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"

namespace module {
namespace behaviour {
    namespace tools {

        using extension::Configuration;

        using PyScript = message::behaviour::PythonScript;
        using message::extension::ExecuteScriptByName;

        using utility::behaviour::ActionPriorites;
        using utility::behaviour::RegisterAction;
        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;

        PythonScript::PythonScript(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), id(size_t(this) * size_t(this) - size_t(this)) {

            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
                id,
                "PythonScript",
                {std::pair<float, std::set<LimbID>>(
                    20, {LimbID::HEAD, LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM})},
                [this](const std::set<LimbID>&) {},
                [this](const std::set<LimbID>&) {},
                [this](const std::set<ServoID>&) {}}));

            // on<Configuration>("PythonScript.yaml").then([this](const Configuration& config) {});

            on<Trigger<PyScript>>().then([this](const PyScript& script) {
                emit(std::make_unique<ExecuteScriptByName>(id,
                                                           std::vector<std::string>({script.script_name}),
                                                           std::vector<double>({1.0}),
                                                           NUClear::clock::now()));
            });
        }
    }  // namespace tools
}  // namespace behaviour
}  // namespace module
