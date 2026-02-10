#include "ServoTest.hpp"

#include <chrono>

#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/actuation/ServoTarget.hpp"
#include "message/actuation/Servos.hpp"
#include "message/input/Sensors.hpp"


namespace module::skill {

    using extension::Configuration;
    using message::actuation::Body;
    using message::actuation::ServoCommand;
    using message::actuation::ServoState;
    using message::actuation::ServoTarget;
    using message::input::Sensors;
    using utility::input::ServoID;
    using utility::support::Expression;

    std::map<uint32_t, std::string> id_to_joint_name = {
        {0, "right_shoulder_pitch"}, {1, "left_shoulder_pitch"}, {2, "right_shoulder_roll"}, {3, "left_shoulder_roll"},
        {4, "right_elbow_pitch"},    {5, "left_elbow_pitch"},    {6, "right_hip_yaw"},       {7, "left_hip_yaw"},
        {8, "right_hip_roll"},       {9, "left_hip_roll"},       {10, "right_hip_pitch"},    {11, "left_hip_pitch"},
        {12, "right_knee_pitch"},    {13, "left_knee_pitch"},    {14, "right_ankle_pitch"},  {15, "left_ankle_pitch"},
        {16, "right_ankle_roll"},    {17, "left_ankle_roll"},    {18, "neck_yaw"},           {19, "head_pitch"}};

    ServoTest::ServoTest(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        joint_map = {
            {0, 7},    // L_HIP_YAW -> left_hip_yaw (ID 7)
            {1, 9},    // L_HIP_ROLL -> left_hip_roll (ID 9)
            {2, 11},   // L_HIP_PITCH -> left_hip_pitch (ID 11)
            {3, 13},   // L_KNEE -> left_knee_pitch (ID 13)
            {4, 15},   // L_ANKLE_PITCH -> left_ankle_pitch (ID 15)
            {5, 17},   // L_ANKLE_ROLL -> left_ankle_roll (ID 17)
            {6, 6},    // R_HIP_YAW -> right_hip_yaw (ID 6)
            {7, 8},    // R_HIP_ROLL -> right_hip_roll (ID 8)
            {8, 10},   // R_HIP_PITCH -> right_hip_pitch (ID 10)
            {9, 12},   // R_KNEE -> right_knee_pitch (ID 12)
            {10, 14},  // R_ANKLE_PITCH -> right_ankle_pitch (ID 14)
            {11, 16},  // R_ANKLE_ROLL -> right_ankle_roll (ID 16)
            {12, 18},  // NECK_YAW -> neck_yaw (ID 18)
            {13, 19},  // HEAD_PITCH -> head_pitch (ID 19)
            {14, 1},   // L_SHOULDER_PITCH -> left_shoulder_pitch (ID 1)
            {15, 3},   // L_SHOULDER_ROLL -> left_shoulder_roll (ID 3)
            {16, 5},   // L_ELBOW -> left_elbow_pitch (ID 5)
            {17, 0},   // R_SHOULDER_PITCH -> right_shoulder_pitch (ID 0)
            {18, 2},   // R_SHOULDER_ROLL -> right_shoulder_roll (ID 2)
            {19, 4}    // R_ELBOW -> right_elbow_pitch (ID 4)
        };

        on<Configuration>("ServoTest.yaml").then([this](const Configuration& config) {
            cfg.test_amplitude = config["test_amplitude"].as<double>();
            cfg.hold_time      = config["hold_time"].as<double>();
            cfg.wait_time      = config["wait_time"].as<double>();
            cfg.auto_start     = config["auto_start"].as<bool>();
            cfg.legs_only      = config["legs_only"].as<bool>();

            default_pose = Eigen::Matrix<double, 20, 1>(config["default_pose"].as<Expression>());

            log<INFO>("Servo test configured");
            log<INFO>("  Test amplitude: ", cfg.test_amplitude, " rad");
            log<INFO>("  Hold time: ", cfg.hold_time, " seconds");
            log<INFO>("  Wait time: ", cfg.wait_time, " seconds");
            log<INFO>("  Legs only: ", cfg.legs_only);

            if (cfg.auto_start) {
                log<INFO>("Auto-starting servo mapping test...");
                current_state       = TestState::MOVING_POSITIVE;
                current_servo_index = 0;
                state_start_time    = NUClear::clock::now();
                test_servo_loop();
            }
        });
    }

    void test_servo_loop(void) {
        while (true) {
            if (current_state == TestState::IDLE) {
                return;
            }
            log<INFO>("Running servo test...");

            auto now     = NUClear::clock::now();
            auto elapsed = std::chrono::duration<double>(now - state_start_time).count();

            // Determine the range of servos to test
            int max_servo_index = cfg.legs_only ? 12 : 20;  // 12 for legs only, 20 for all

            switch (current_state) {
                case TestState::MOVING_POSITIVE: {
                    uint32_t hardware_id = joint_map[current_servo_index].second;
                    log<INFO>("════════════════════════════════════════════════════════");
                    log<INFO>("Testing Servo ", current_servo_index);
                    log<INFO>("  Hardware ID: ", hardware_id);
                    log<INFO>("  Name: ", id_to_joint_name[hardware_id]);
                    log<INFO>("  Moving to +", cfg.test_amplitude, " rad from default");
                    log<INFO>("════════════════════════════════════════════════════════");

                    emit_test_commands();
                    current_state    = TestState::HOLDING_POSITIVE;
                    state_start_time = now;
                    break;
                }

                case TestState::HOLDING_POSITIVE: {
                    if (elapsed >= cfg.hold_time) {
                        log<INFO>("Moving to -", cfg.test_amplitude, " rad from default");
                        current_state    = TestState::MOVING_NEGATIVE;
                        state_start_time = now;
                    }
                    break;
                }

                case TestState::MOVING_NEGATIVE: {
                    emit_test_commands();
                    current_state    = TestState::HOLDING_NEGATIVE;
                    state_start_time = now;
                    break;
                }

                case TestState::HOLDING_NEGATIVE: {
                    if (elapsed >= cfg.hold_time) {
                        log<INFO>("Returning to default position");
                        current_state    = TestState::RETURNING;
                        state_start_time = now;
                    }
                    break;
                }

                case TestState::RETURNING: {
                    emit_test_commands();
                    if (elapsed >= cfg.wait_time) {
                        current_servo_index++;
                        if (current_servo_index >= max_servo_index) {
                            log<INFO>("════════════════════════════════════════════════════════");
                            log<INFO>("Servo mapping test complete!");
                            log<INFO>("════════════════════════════════════════════════════════");
                            current_state       = TestState::IDLE;
                            current_servo_index = 0;
                        }
                        else {
                            current_state    = TestState::MOVING_POSITIVE;
                            state_start_time = now;
                        }
                    }
                    break;
                }

                case TestState::IDLE: break;
            }
        }
    }


    void ServoTest::emit_test_commands() {
        uint32_t hardware_id   = joint_map[current_servo_index].second;
        double target_position = default_pose[current_servo_index];

        switch (current_state) {
            case TestState::MOVING_POSITIVE:
            case TestState::HOLDING_POSITIVE: target_position += cfg.test_amplitude; break;
            case TestState::MOVING_NEGATIVE:
            case TestState::HOLDING_NEGATIVE: target_position -= cfg.test_amplitude; break;
            case TestState::RETURNING:
            default: break;
        }

        log<INFO>("Emitting ServoTarget: ID=",
                  hardware_id,
                  " Name=",
                  id_to_joint_name[hardware_id],
                  " Position=",
                  target_position);

        auto target = std::make_unique<ServoTarget>();

        // Set the timestamp - use NUClear clock directly
        target->time = NUClear::clock::now();

        target->id       = hardware_id;
        target->name     = id_to_joint_name[hardware_id];
        target->position = static_cast<float>(target_position);
        target->gain     = 50.0f;   // Medium stiffness
        target->torque   = 100.0f;  // Full torque

        emit(target);
    }


}  // namespace module::skill
