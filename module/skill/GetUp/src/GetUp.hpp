#ifndef MODULE_SKILL_GETUP_HPP
#define MODULE_SKILL_GETUP_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"
#include "extension/behaviour/Script.hpp"

namespace module::skill {

    class GetUp : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            std::vector<::extension::behaviour::ScriptRequest> getup_front;
            std::vector<::extension::behaviour::ScriptRequest> getup_back;
            std::vector<::extension::behaviour::ScriptRequest> getup_right;
            std::vector<::extension::behaviour::ScriptRequest> getup_left;
            std::vector<::extension::behaviour::ScriptRequest> getup_upright;
            std::vector<::extension::behaviour::ScriptRequest> getup_inverted;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the GetUp reactor.
        explicit GetUp(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_GETUP_HPP
