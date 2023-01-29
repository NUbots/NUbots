#include "Dive.hpp"

#include <nuclear>

#include "extension/Configuration.hpp"
#include "extension/Script.hpp"

#include "message/behaviour/Dive.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"


namespace module::behaviour::skills {

    using extension::Configuration;
    using extension::ExecuteScriptByName;

    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;

    using utility::behaviour::ActionPriorities;
    using utility::behaviour::RegisterAction;

    Dive::Dive(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Dive.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Dive.yaml
            this->log_level   = config["log_level"].as<NUClear::LogLevel>();
            cfg.dive_priority = config["dive_priority"].as<float>();
        });

        on<Trigger<message::behaviour::Dive>>().then([this](const message::behaviour::Dive& dive) {
            dive_left = dive.left;
            updatePriority(cfg.dive_priority);
        });

        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
            subsumption_id,
            "Dive",
            {std::pair<float, std::set<LimbID>>(
                0,
                {LimbID::HEAD, LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM})},
            [this](const std::set<LimbID>& /*unused*/) {
                if (dive_left) {
                    emit(std::make_unique<ExecuteScriptByName>(subsumption_id,
                                                               std::vector<std::string>({"DiveLeft.yaml"})));
                }
                else {
                    emit(std::make_unique<ExecuteScriptByName>(subsumption_id,
                                                               std::vector<std::string>({"DiveRight.yaml"})));
                }
            },
            [this](const std::set<LimbID>& /*unused*/) { updatePriority(0); },
            [this](const std::set<ServoID>& /*unused*/) { updatePriority(0); }}));
    }

    void Dive::updatePriority(const float& priority) {
        emit(std::make_unique<ActionPriorities>(ActionPriorities{subsumption_id, {priority}}));
    }

}  // namespace module::behaviour::skills
