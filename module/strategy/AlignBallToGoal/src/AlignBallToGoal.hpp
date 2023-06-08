#ifndef MODULE_STRATEGY_ALIGNBALLTOGOAL_HPP
#define MODULE_STRATEGY_ALIGNBALLTOGOAL_HPP

#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class AlignBallToGoal : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief How close the ball needs to be before we start aligning
            double ball_distance_threshold = 0.0;
            /// @brief If the angle to the goal is between the threshold and 0, we stop aligning
            double angle_threshold = 0.0;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the AlignBallToGoal reactor.
        explicit AlignBallToGoal(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_ALIGNBALLTOGOAL_HPP
