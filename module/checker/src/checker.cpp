#include "checker.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/onboarding.hpp"

#include "utility/skill/Script.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module {

    using extension::Configuration;
    using extension::behaviour::Task;
    using message::actuation::HeadSequence;
    using message::onboarding::check;
    using utility::skill::load_script;
    using utility::support::Expression;

    checker::checker(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("checker.yaml").then([this](const Configuration& config) {
            // Use configuration here from file checker.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Trigger<check>>().then([this](const check& c) {
            log<INFO>("Received check with value =", c.value);
            if (c.value == 55 && !validated) {
                validated = true;
                log<INFO>("The value is correct!");
                emit<Task>(load_script<HeadSequence>("NodYes.yaml"));
            }
            else if (!validated) {
                log<ERROR>("The value is incorrect!");
            }
            else if (validated) {
                log<INFO>("Already validated, ignoring further checks.");
            }
        });
    }

}  // namespace module
