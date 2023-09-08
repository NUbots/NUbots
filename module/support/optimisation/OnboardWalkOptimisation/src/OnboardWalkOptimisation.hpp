#ifndef MODULE_SUPPORT_OPTIMISATION_ONBOARDWALKOPTIMISATION_HPP
#define MODULE_SUPPORT_OPTIMISATION_ONBOARDWALKOPTIMISATION_HPP

#include <nuclear>

namespace module::support::optimisation {

    class OnboardWalkOptimisation : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Threshold angle for standing stably, from torso z axis to world z axis.
            float standing_angle = 0.0f;

            /// @brief Time required to be standing before starting a new individual.
            float wait_time = 0.0f;
        } cfg;

        /// @brief Indicates that the optimisation needs resetting for the next individual.
        bool resetting = false;

        /// @brief Used for stable stand, indicates whether we are still standing upright.
        bool is_upright = false;

        /// @brief Used to make sure the robot has been standing for enough time before starting.
        NUClear::clock::time_point start_time{NUClear::clock::now()};

    public:
        /// @brief Called by the powerplant to build and setup the OnboardWalkOptimisation reactor.
        explicit OnboardWalkOptimisation(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::support::optimisation

#endif  // MODULE_SUPPORT_OPTIMISATION_ONBOARDWALKOPTIMISATION_HPP
