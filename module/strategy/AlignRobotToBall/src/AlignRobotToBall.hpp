#ifndef MODULE_STRATEGY_ALIGNROBOTTOBALL_HPP
#define MODULE_STRATEGY_ALIGNROBOTTOBALL_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class AlignRobotToBall : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Angle threshold to start rotating to face the ball
            double start_rotating_angle_threshold = 0.0;
            /// @brief Angle threshold to stop rotating to face the ball
            double stop_rotating_angle_threshold = 0.0;
            /// @brief How far away the ball needs to be to align
            double ball_distance_threshold = 0.0;
        } cfg;

        /// @brief Whether the robot is currently rotating to face the ball
        bool rotate = false;

    public:
        /// @brief Called by the powerplant to build and setup the AlignRobotToBall reactor.
        explicit AlignRobotToBall(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_ALIGNROBOTTOBALL_HPP
