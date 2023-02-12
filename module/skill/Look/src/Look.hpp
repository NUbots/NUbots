#ifndef MODULE_SKILL_LOOK_HPP
#define MODULE_SKILL_LOOK_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::skill {

    class Look : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            float smoothing_factor = 0.0;
            float head_gain        = 0.0;
            float head_torque      = 0.0;
        } cfg;

        /// @brief Whether to smooth the head movements
        bool smooth = false;

    public:
        /// @brief Called by the powerplant to build and setup the Look reactor.
        explicit Look(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_LOOK_HPP
