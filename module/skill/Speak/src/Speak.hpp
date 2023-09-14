#ifndef MODULE_SKILL_SPEAK_HPP
#define MODULE_SKILL_SPEAK_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

namespace module::skill {

    class Speak : public ::extension::behaviour::BehaviourReactor {
        /// @brief Stores configuration values
        struct Config {
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the Speak reactor.
        explicit Speak(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_SPEAK_HPP
