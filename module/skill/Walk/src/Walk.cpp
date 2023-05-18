#include "Walk.hpp"

#include <fmt/format.h>

#include "extension/Configuration.hpp"

#include "message/actuation/KinematicsModel.hpp"
#include "message/actuation/Limbs.hpp"
#include "message/actuation/LimbsIK.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/behaviour/Behaviour.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/eye/DataPoint.hpp"
#include "message/motion/GetupCommand.hpp"
#include "message/skill/Walk.hpp"

#include "utility/actuation/InverseKinematics.hpp"
#include "utility/math/comparison.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::skill {

    using extension::Configuration;

    using message::actuation::KinematicsModel;
    using message::actuation::LeftArm;
    using message::actuation::LeftLegIK;
    using message::actuation::RightArm;
    using message::actuation::RightLegIK;
    using message::actuation::ServoCommand;
    using message::actuation::ServoState;
    using message::behaviour::Behaviour;
    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;

    using message::input::Sensors;
    using WalkTask = message::skill::Walk;

    using utility::actuation::kinematics::calculateLegJoints;
    using utility::input::ServoID;
    using utility::math::euler::EulerIntrinsicToMatrix;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::nusight::graph;
    using utility::support::Expression;


    Walk::Walk(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Walk.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            // Configure the motion generation options
            utility::skill::MotionGenerationOptions<float> walk_engine_options;
            walk_engine_options.step_limits           = config["step_limits"].as<Expression>();
            walk_engine_options.step_height           = config["step_height"].as<float>();
            walk_engine_options.step_period           = config["step_period"].as<float>();
            walk_engine_options.step_width            = config["step_width"].as<float>();
            walk_engine_options.torso_height          = config["torso_height"].as<float>();
            walk_engine_options.torso_pitch           = config["torso_pitch"].as<float>();
            walk_engine_options.torso_midpoint_offset = config["torso_midpoint_offset"].as<Expression>();
            walk_engine.configure(walk_engine_options);
            walk_engine.reset();
            last_update_time = NUClear::clock::now();

            for (int id = 0; id < ServoID::NUMBER_OF_SERVOS; ++id) {
                // Sets the leg gains
                if ((id >= ServoID::R_HIP_YAW) && (id < ServoID::HEAD_YAW)) {
                    cfg.servo_states[id] = ServoState(config["gains"]["legs"].as<float>(), 100);
                }
                // Sets the arm gains
                if (id < ServoID::R_HIP_YAW) {
                    cfg.servo_states[id] = ServoState(config["gains"]["arms"].as<float>(), 100);
                }
            }

            cfg.arm_positions.emplace_back(ServoID::R_SHOULDER_PITCH,
                                           config["arms"]["right_shoulder_pitch"].as<float>());
            cfg.arm_positions.emplace_back(ServoID::L_SHOULDER_PITCH,
                                           config["arms"]["left_shoulder_pitch"].as<float>());
            cfg.arm_positions.emplace_back(ServoID::R_SHOULDER_ROLL, config["arms"]["right_shoulder_roll"].as<float>());
            cfg.arm_positions.emplace_back(ServoID::L_SHOULDER_ROLL, config["arms"]["left_shoulder_roll"].as<float>());
            cfg.arm_positions.emplace_back(ServoID::R_ELBOW, config["arms"]["right_elbow"].as<float>());
            cfg.arm_positions.emplace_back(ServoID::L_ELBOW, config["arms"]["left_elbow"].as<float>());
        });

        // Runs every time the Walk provider starts (wasn't running)
        on<Start<WalkTask>>().then([this]() {
            // Reset the walk engine
            first_run = true;
            walk_engine.reset();
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3f::Zero()));
        });

        // Runs every time the Walk task is removed from the director tree
        on<Stop<WalkTask>>().then(
            [this] { emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3f::Zero())); });

        // MAIN LOOP
        on<Provide<WalkTask>,
           Needs<LeftLegIK>,
           Needs<RightLegIK>,
           Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>,
           Single>()
            .then([this](const WalkTask& walk) {
                const float dt = get_time_delta();
                switch (walk_engine.update(dt, walk.velocity_target).value) {
                    case WalkState::State::WALKING:
                    case WalkState::State::STOPPING:
                        update_desired_pose();
                        emit(std::make_unique<Stability>(Stability::DYNAMIC));
                        break;
                    case WalkState::State::STOPPED: emit(std::make_unique<Stability>(Stability::STANDING)); break;
                    case WalkState::State::UNKNOWN:
                    default: NUClear::log<NUClear::WARN>("Unknown state"); break;
                }
                // Emit the walking state
                emit(std::make_unique<WalkState>(walk_engine.get_state(), Eigen::Vector3f::Zero()));
            });

        // Stand Reaction - Sets walk_engine commands to zero, checks walk_engine state, Sets stability state
        on<Provide<WalkTask>,
           Needs<LeftLegIK>,
           Needs<RightLegIK>,
           Causing<Stability, Stability::STANDING>,
           Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>,
           Single>()
            .then([this] {
                // Stop the walk engine (request zero velocity)
                const float dt = get_time_delta();
                switch (walk_engine.update(dt, Eigen::Vector3f::Zero()).value) {
                    case WalkState::State::STOPPED: emit(std::make_unique<Stability>(Stability::STANDING)); break;
                    case WalkState::State::STOPPING:
                        update_desired_pose();
                        emit(std::make_unique<Stability>(Stability::DYNAMIC));
                        break;
                    case WalkState::State::WALKING:
                        log<NUClear::WARN>("Walk engine state should be either STOPPING or STOPPED");
                        break;
                    case WalkState::State::UNKNOWN:
                    default: NUClear::log<NUClear::WARN>("Unknown state"); break;
                }

                // Emit the walking state
                emit(std::make_unique<WalkState>(walk_engine.get_state(), Eigen::Vector3f::Zero()));
            });
    }

    float Walk::get_time_delta() {
        // Time is wrong when we run it for the first time
        float time_delta = 0;
        if (first_run) {
            first_run  = false;
            time_delta = 1.0f / UPDATE_FREQUENCY;
        }
        else {
            // Compute time delta
            const auto current_time = NUClear::clock::now();
            time_delta =
                std::chrono::duration_cast<std::chrono::duration<float>>(current_time - last_update_time).count();
            last_update_time = current_time;
        }
        return time_delta;
    }

    void Walk::update_desired_pose() {
        // Compute the goal position time
        const NUClear::clock::time_point time = NUClear::clock::now() + Per<std::chrono::seconds>(UPDATE_FREQUENCY);

        // Get desired feet poses in the torso {t} frame
        Eigen::Transform<float, 3, Eigen::Isometry> Htl = walk_engine.get_foot_pose(true);
        Eigen::Transform<float, 3, Eigen::Isometry> Htr = walk_engine.get_foot_pose(false);

        // Legs
        auto left_leg   = std::make_unique<LeftLegIK>();
        left_leg->time  = time;
        left_leg->Htl   = Htl.cast<double>().matrix();
        auto right_leg  = std::make_unique<RightLegIK>();
        right_leg->time = time;
        right_leg->Htr  = Htr.cast<double>().matrix();
        // Arms
        auto left_arm  = std::make_unique<LeftArm>();
        auto right_arm = std::make_unique<RightArm>();

        // Loop to set the servo states
        for (int id = 0; id < ServoID::NUMBER_OF_SERVOS - 2; ++id) {
            // Set the legs
            if ((id >= ServoID::R_HIP_YAW) && (id % 2 == 0)) {  // right legs
                right_leg->servos[id] = cfg.servo_states[ServoID(id)];
            }
            else if ((id >= ServoID::R_HIP_YAW) && (id % 2 == 1)) {  // left legs

                left_leg->servos[id] = cfg.servo_states[ServoID(id)];
            }
            else if ((id < ServoID::R_HIP_YAW) && (id % 2 == 0)) {  // right arms
                right_arm->servos[id] =
                    ServoCommand(time, cfg.arm_positions[ServoID(id)].second, cfg.servo_states[ServoID(id)]);
            }
            else if ((id < ServoID::R_HIP_YAW) && (id % 2 == 1)) {  // left arms
                left_arm->servos[id] =
                    ServoCommand(time, cfg.arm_positions[ServoID(id)].second, cfg.servo_states[ServoID(id)]);
            }
        }

        emit<Task>(left_leg, 0, false, "Walk left leg");
        emit<Task>(right_leg, 0, false, "Walk right leg");
        emit<Task>(left_arm, 0, true, "Walk left arm");
        emit<Task>(right_arm, 0, true, "Walk right arm");

        // Plot graphs of desired trajectories
        if (log_level <= NUClear::DEBUG) {
            Eigen::Vector3f thetaTL = MatrixToEulerIntrinsic(Htl.linear());
            emit(graph("Left foot desired position (x,y,z)", Htl(0, 3), Htl(1, 3), Htl(2, 3)));
            emit(graph("Left foot desired orientation (r,p,y)", thetaTL.x(), thetaTL.y(), thetaTL.z()));
            Eigen::Vector3f thetaTR = MatrixToEulerIntrinsic(Htr.linear());
            emit(graph("Right foot desired position (x,y,z)", Htr(0, 3), Htr(1, 3), Htr(2, 3)));
            emit(graph("Right foot desired orientation (r,p,y)", thetaTR.x(), thetaTR.y(), thetaTR.z()));
        }
    }

}  // namespace module::skill
