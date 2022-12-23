#ifndef MODULE_SKILL_KICKSCRIPT_HPP
#define MODULE_SKILL_KICKSCRIPT_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::skill {

    class KickScript : public ::extension::behaviour::BehaviourReactor {
    public:
        /// @brief Called by the powerplant to build and setup the KickScript reactor.
        explicit KickScript(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_KICKSCRIPT_HPP