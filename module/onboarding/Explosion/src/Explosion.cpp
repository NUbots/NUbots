#include "Explosion.hpp"

#include "extension/Configuration.hpp"




//
//#include "message/actuation/Limbs.hpp"
//#include "message/skill/NodYes.hpp"
//
//#include "extension/Behaviour.hpp"
//
//#include "utility/input/LimbID.hpp"
//#include "utility/skill/Script.hpp"
//
//


#include "message/onboarding/Catalyst.hpp"
#include "message/onboarding/Reaction.hpp"
#include "message/onboarding/Explosion.hpp"

namespace module::onboarding {

    using extension::Configuration;
    using utility::skill::load_script;

    Explosion::Explosion(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        using message::onboarding::Catalyst;
        using message::onboarding::Reaction;
        using message::onboarding::Explosion;

        on<Configuration>("Explosion.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Explosion.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Trigger<Reaction>>().then([this](const Reaction& react_msg) {
            if(react_msg.count == 10 && react_msg.sum == 55) {
                emit<Task>(load_script<LimbsSequence>({"NodYes.yaml"}));
            }
        });
    }

}  // namespace module::onboarding
