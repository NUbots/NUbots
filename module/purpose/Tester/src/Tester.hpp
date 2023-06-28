#ifndef MODULE_PURPOSE_TESTER_HPP
#define MODULE_PURPOSE_TESTER_HPP

#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    class Tester : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Whether to emit FindBall task
            bool find_ball = false;
            /// @brief Priority of FindBall task
            int find_ball_priority = 1;
            /// @brief Whether to emit LookAtBall task
            bool look_at_ball = false;
            /// @brief Priority of LookAtBall task
            int look_at_ball_priority = 1;
            /// @brief Whether to emit StandStill task
            bool walk_to_ball = false;
            /// @brief Priority of StandStill task
            int walk_to_ball_priority = 1;
            /// @brief Whether to emit AlignBallToGoal task
            bool align_ball_to_goal = false;
            /// @brief Priority of AlignBallToGoal task
            int align_ball_to_goal_priority = 1;
            /// @brief Whether to emit KickToGoal task
            bool kick_to_goal = false;
            /// @brief Priority of KickToGoal task
            int kick_to_goal_priority = 1;
            /// @brief Whether to emit WalkToFieldPosition task
            bool walk_to_field_position = false;
            /// @brief Priority of WalkToFieldPosition task
            int walk_to_field_position_priority = 1;
            /// @brief Whether to emit KickTo task
            bool kick_to = false;
            /// @brief Priority of KickTo task
            int kick_to_priority = 1;
            /// @brief Whether to emit LookAround task
            bool look_around = false;
            /// @brief Priority of LookAround task
            int look_around_priority = 1;
            /// Whether to emit StandStill task
            bool stand_still = false;
            /// @brief Priority of StandStill task
            int stand_still_priority = 1;

            /// @brief Position to walk to when emitting WalkToFieldPosition task
            Eigen::Vector3f walk_to_field_position_position = Eigen::Vector3f::Zero();
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the Tester reactor.
        explicit Tester(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_TESTER_HPP
