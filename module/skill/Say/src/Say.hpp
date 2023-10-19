#ifndef MODULE_SKILL_SAY_HPP
#define MODULE_SKILL_SAY_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

namespace module::skill {

    class Say : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            std::string voice = "en_US/vctk_low";
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the Say reactor.
        explicit Say(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_SAY_HPP
