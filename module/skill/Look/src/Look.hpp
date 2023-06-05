#ifndef MODULE_SKILL_LOOK_HPP
#define MODULE_SKILL_LOOK_HPP

#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::skill {

    class Look : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            float smoothing_factor = 0.0f;
            float head_gain        = 0.0f;
            float head_torque      = 0.0f;
        } cfg;

        /// @brief Store whether we are smoothing the head movements from the previous run, to help with transitioning
        /// between smoothing and not smoothing. Smoothing is given by the message.
        bool smooth = false;

        /// @brief Last goal vector, used for smoothing
        Eigen::Vector3f uPCt = Eigen::Vector3f::Zero();

    public:
        /// @brief Called by the powerplant to build and setup the Look reactor.
        explicit Look(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_LOOK_HPP
