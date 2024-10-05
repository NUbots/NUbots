#include "Judge.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/onboarding/Judge.hpp"
#include "message/onboarding/Ping.hpp"

#include "utility/skill/Script.hpp"
namespace module::onboarding {

    using extension::Configuration;

    Judge::Judge(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        using extension::behaviour::Task;
        using message::actuation::BodySequence;
        using message::onboarding::Judge;
        using message::onboarding::Ping;
        using utility::skill::load_script;

        on<Configuration>("Judge.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Judge.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this] {});

        on<Trigger<Ping>>().then([this](const Ping& ping_msg) {
            auto sol = (10 * 11) / 2;
            if (sol == ping_msg.val) {
                log<NUClear::INFO>("Nodding");
                emit<Task>(load_script<BodySequence>("NodYes.yaml"));
            }
        });
    }

}  // namespace module::onboarding
