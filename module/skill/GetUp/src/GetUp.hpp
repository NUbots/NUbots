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
            /// @brief Script sequence to run when getting from lying on the front to standing
            std::vector<::extension::behaviour::ScriptRequest> getup_front;
            /// @brief Script sequence to run when getting from lying on the back to standing
            std::vector<::extension::behaviour::ScriptRequest> getup_back;
            /// @brief Script sequence to run when getting from lying on the right to standing
            std::vector<::extension::behaviour::ScriptRequest> getup_right;
            /// @brief Script sequence to run when getting from lying on the left to standing
            std::vector<::extension::behaviour::ScriptRequest> getup_left;
            /// @brief Script sequence to run when already upright
            std::vector<::extension::behaviour::ScriptRequest> getup_upright;
            /// @brief Script sequence to run when told to get up while upside down
            std::vector<::extension::behaviour::ScriptRequest> getup_inverted;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the GetUp reactor.
        explicit GetUp(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_GETUP_HPP
