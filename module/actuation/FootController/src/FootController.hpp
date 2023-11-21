#ifndef MODULE_ACTUATION_FOOTCONTROLLER_HPP
#define MODULE_ACTUATION_FOOTCONTROLLER_HPP

#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::actuation {

    class FootController : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Gains for the servos
            double servo_gain = 0.0;
            /// @brief Whether or not to level the foot with the ground
            bool keep_level = false;

            /// @brief COM error gain
            double K_com = 0.0;

            /// @brief ZMP error gain
            double K_zmp = 0.0;

            /// @brief Velocity error gain
            double K_vel = 0.0;

            /// @brief End of step error gain
            double K_eos = 0.0;

            /// @brief Exponential filter gain
            double alpha = 0.0;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the FootController reactor.
        explicit FootController(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Torso x-y offset in planted foot space
        Eigen::Vector2d torso_offset = Eigen::Vector2d::Zero();

        /// @brief Last time we updated the left foot controller
        NUClear::clock::time_point left_last_update_time{};

        /// @brief Last time we updated the right foot controller
        NUClear::clock::time_point right_last_update_time{};
    };

}  // namespace module::actuation

#endif  // MODULE_ACTUATION_FOOTCONTROLLER_HPP
