#ifndef MODULE_STRATEGY_LOCALISE_HPP
#define MODULE_STRATEGY_LOCALISE_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class Localise : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Uncertainty value field localisation needs to be above to localise.
            double uncertainty_threshold = 0;

            /// @brief Amount of time the robot can be lost before requesting localisation to reset.
            double max_lost_time = 0;

            /// @brief Whether or not to look around when lost.
            bool look_around = false;
        } cfg;

        /// @brief The time point at which the robot became lost.
        NUClear::clock::time_point time_point_lost;

        /// @brief Flag to indicate if the robot has just now become lost.
        bool just_lost = false;

    public:
        /// @brief Called by the powerplant to build and setup the Localise reactor.
        explicit Localise(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_LOCALISE_HPP
