#include "Walk.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/actuation/LimbsIK.hpp"
#include "message/behaviour/Behaviour.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/eye/DataPoint.hpp"
#include "message/skill/ControlFoot.hpp"
#include "message/skill/Walk.hpp"

#include "utility/actuation/InverseKinematics.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::skill {

    using extension::Configuration;

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
    using message::skill::ControlLeftFoot;
    using message::skill::ControlRightFoot;
    using WalkTask = message::skill::Walk;

    using utility::actuation::kinematics::calculateLegJoints;
    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::math::euler::MatrixToEulerIntrinsic;
    using utility::nusight::graph;
    using utility::support::Expression;

    Walk::Walk(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Walk.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            // Configure the motion generation options
            utility::skill::MotionGenerationOptions<float> walk_engine_options;
            walk_engine_options.step_period       = config["walk"]["period"].as<float>();
            walk_engine_options.step_apex_ratio   = config["walk"]["step"]["apex_ratio"].as<float>();
            walk_engine_options.step_limits       = config["walk"]["step"]["limits"].as<Expression>();
            walk_engine_options.step_height       = config["walk"]["step"]["height"].as<float>();
            walk_engine_options.step_width        = config["walk"]["step"]["width"].as<float>();
            walk_engine_options.torso_height      = config["walk"]["torso"]["height"].as<float>();
            walk_engine_options.torso_pitch       = config["walk"]["torso"]["pitch"].as<float>();
            walk_engine_options.torso_sway_offset = config["walk"]["torso"]["sway_offset"].as<Expression>();
            walk_engine_options.torso_sway_ratio  = config["walk"]["torso"]["sway_ratio"].as<float>();
            walk_engine.configure(walk_engine_options);
            walk_engine.reset();
            last_update_time = NUClear::clock::now();

            for (auto id : utility::input::LimbID::servos_for_legs()) {
                cfg.servo_states[id] = ServoState(config["gains"]["legs"].as<float>(), 100);
            }

            for (auto id : utility::input::LimbID::servos_for_arms()) {
                cfg.servo_states[id] = ServoState(config["gains"]["arms"].as<float>(), 100);
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

        // Start - Runs every time the Walk provider starts (wasn't running)
        on<Start<WalkTask>>().then([this]() {
            last_update_time = NUClear::clock::now();
            walk_engine.reset();
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3f::Zero()));
        });

        // Stop - Runs every time the Walk task is removed from the director tree
        on<Stop<WalkTask>>().then(
            [this] { emit(std::make_unique<WalkState>(WalkState::State::STOPPED, Eigen::Vector3f::Zero())); });

        // Main loop - Updates the walk engine at fixed frequency of UPDATE_FREQUENCY
        on<Provide<WalkTask>,
           Needs<LeftLegIK>,
           Needs<RightLegIK>,
           Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>,
           Single>()
            .then([this](const WalkTask& walk_task) {
                switch (walk_engine.update(compute_time_delta(), walk_task.velocity_target).value) {
                    case WalkState::State::WALKING:
                    case WalkState::State::STOPPING: walk(); break;
                    case WalkState::State::STOPPED: emit(std::make_unique<Stability>(Stability::STANDING)); break;
                    case WalkState::State::UNKNOWN:
                    default: NUClear::log<NUClear::WARN>("Unknown state."); break;
                }
                // Emit the walking state
                emit(std::make_unique<WalkState>(walk_engine.get_state(), Eigen::Vector3f::Zero()));
            });

        // Stand Reaction - Sets walk_engine commands to zero, checks walk engine state and sets stability state
        on<Provide<WalkTask>,
           Needs<LeftLegIK>,
           Needs<RightLegIK>,
           Causing<Stability, Stability::STANDING>,
           Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>,
           Single>()
            .then([this] {
                // Stop the walk engine (request zero velocity)
                switch (walk_engine.update(compute_time_delta(), Eigen::Vector3f::Zero()).value) {
                    case WalkState::State::STOPPED: emit(std::make_unique<Stability>(Stability::STANDING)); break;
                    case WalkState::State::STOPPING: walk(); break;
                    case WalkState::State::WALKING: log<NUClear::WARN>("Walk engine state shouldn't be here."); break;
                    case WalkState::State::UNKNOWN:
                    default: NUClear::log<NUClear::WARN>("Unknown state"); break;
                }

                // Emit the walking state
                emit(std::make_unique<WalkState>(walk_engine.get_state(), Eigen::Vector3f::Zero()));
            });
    }

    float Walk::compute_time_delta() {
        auto time_delta =
            std::chrono::duration_cast<std::chrono::duration<float>>(NUClear::clock::now() - last_update_time).count();
        last_update_time = NUClear::clock::now();
        return time_delta;
    }

    void Walk::walk() {
        // Compute the goal position time
        const NUClear::clock::time_point time = NUClear::clock::now() + Per<std::chrono::seconds>(UPDATE_FREQUENCY);

        // Get desired feet poses in the torso {t} frame from the walk engine
        Eigen::Transform<float, 3, Eigen::Isometry> Htl = walk_engine.get_foot_pose(LimbID::LEFT_LEG);
        Eigen::Transform<float, 3, Eigen::Isometry> Htr = walk_engine.get_foot_pose(LimbID::RIGHT_LEG);

        emit<Task>(std::make_unique<ControlLeftFoot>(Htl.matrix(), time));
        emit<Task>(std::make_unique<ControlRightFoot>(Htr.matrix(), time));

        // Construct Arm IK tasks
        auto left_arm  = std::make_unique<LeftArm>();
        auto right_arm = std::make_unique<RightArm>();
        for (auto id : utility::input::LimbID::servos_for_limb(LimbID::RIGHT_ARM)) {
            right_arm->servos[id] =
                ServoCommand(time, cfg.arm_positions[ServoID(id)].second, cfg.servo_states[ServoID(id)]);
        }
        for (auto id : utility::input::LimbID::servos_for_limb(LimbID::LEFT_ARM)) {
            left_arm->servos[id] =
                ServoCommand(time, cfg.arm_positions[ServoID(id)].second, cfg.servo_states[ServoID(id)]);
        }

        emit<Task>(left_arm, 0, true, "Walk left arm");
        emit<Task>(right_arm, 0, true, "Walk right arm");

        // Emit stability state of dynamic
        emit(std::make_unique<Stability>(Stability::DYNAMIC));

        // Plot the desired feet poses in the torso {t} frame
        if (log_level <= NUClear::DEBUG) {
            Eigen::Vector3f thetaTL = MatrixToEulerIntrinsic(Htl.linear());
            emit(graph("Left foot desired position (x,y,z)", Htl(0, 3), Htl(1, 3), Htl(2, 3)));
            emit(graph("Left foot desired orientation (r,p,y)", thetaTL.x(), thetaTL.y(), thetaTL.z()));
            Eigen::Vector3f thetaTR = MatrixToEulerIntrinsic(Htr.linear());
            emit(graph("Right foot desired position (x,y,z)", Htr(0, 3), Htr(1, 3), Htr(2, 3)));
            emit(graph("Right foot desired orientation (r,p,y)", thetaTR.x(), thetaTR.y(), thetaTR.z()));
            Eigen::Isometry3f Hpt   = walk_engine.get_torso_pose();
            Eigen::Vector3f thetaPT = MatrixToEulerIntrinsic(Hpt.linear());
            emit(graph("Torso desired position (x,y,z)",
                       Hpt.translation().x(),
                       Hpt.translation().y(),
                       Hpt.translation().z()));
            emit(graph("Torso desired orientation (r,p,y)", thetaPT.x(), thetaPT.y(), thetaPT.z()));
        }
    }

}  // namespace module::skill
