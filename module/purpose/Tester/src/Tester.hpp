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
            /// @brief Priority of FindBall task
            int find_ball_priority = 0;
            /// @brief Priority of LookAtBall task
            int look_at_ball_priority = 0;
            /// @brief Priority of StandStill task
            int walk_to_ball_priority = 0;
            /// @brief Priority of AlignBallToGoal task
            int align_ball_to_goal_priority = 0;
            /// @brief Priority of AlignRobotToBall task
            int align_robot_to_ball_priority = 0;
            /// @brief Priority of KickToGoal task
            int kick_to_goal_priority = 0;
            /// @brief Priority of WalkToFieldPosition task
            int walk_to_field_position_priority = 0;
            /// @brief Priority of KickTo task
            int kick_to_priority = 0;
            /// @brief Priority of LookAround task
            int look_around_priority = 0;
            /// @brief Priority of StandStill task
            int stand_still_priority = 0;

            /// @brief Position to walk to when emitting WalkToFieldPosition task
            Eigen::Vector3f walk_to_field_position_position = Eigen::Vector3f::Zero();
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the Tester reactor.
        explicit Tester(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_TESTER_HPP
