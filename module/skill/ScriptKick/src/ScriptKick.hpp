#ifndef MODULE_SKILL_SCRIPTKICK_HPP
#define MODULE_SKILL_SCRIPTKICK_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::skill {

    class ScriptKick : public ::extension::behaviour::BehaviourReactor {
    public:
        /// @brief Called by the powerplant to build and setup the ScriptKick reactor.
        explicit ScriptKick(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_SCRIPTKICK_HPP
