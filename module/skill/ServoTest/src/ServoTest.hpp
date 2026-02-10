#ifndef MODULE_SKILL_SERVOTEST_HPP
#define MODULE_SKILL_SERVOTEST_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::skill {

    class ServoTest : public ::extension::behaviour::BehaviourReactor {
    private:
        struct Config {
            double test_amplitude = 0.5;
            double hold_time      = 2.0;
            double wait_time      = 1.0;
            bool auto_start       = false;
            bool legs_only        = false;
        } cfg;

        /// Current test state
        enum class TestState { IDLE, MOVING_POSITIVE, HOLDING_POSITIVE, MOVING_NEGATIVE, HOLDING_NEGATIVE, RETURNING };

        TestState current_state = TestState::IDLE;
        int current_servo_index = 0;
        NUClear::clock::time_point state_start_time;

        /// Joint mapping (same as RLWalk)
        std::vector<std::pair<int, uint32_t>> joint_map;

        /// Default pose
        Eigen::Matrix<double, 20, 1> default_pose;

        /// Get servo name from ServoID
        std::string get_servo_name(utility::input::ServoID id) const;

        /// Emit servo commands for current test state
        void emit_test_commands();

        void test_servo_loop();

    public:
        /// Called by the powerplant to build and setup the ServoTest reactor.
        explicit ServoTest(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::skill

#endif  // MODULE_SKILL_SERVOTEST_HPP
