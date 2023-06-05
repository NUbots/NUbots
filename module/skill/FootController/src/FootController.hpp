#ifndef MODULE_SKILL_FOOTCONTROLLER_HPP
#define MODULE_SKILL_FOOTCONTROLLER_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::skill {

    class FootController : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            float servo_gain = 0.0f;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the FootController reactor.
        explicit FootController(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_FOOTCONTROLLER_HPP
