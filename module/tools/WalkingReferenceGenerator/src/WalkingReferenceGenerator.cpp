#include "WalkingReferenceGenerator.hpp"

#include <fstream>

#include "extension/Configuration.hpp"

#include "message/actuation/KinematicsModel.hpp"
#include "message/actuation/Limbs.hpp"
#include "message/actuation/LimbsIK.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/eye/DataPoint.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/ControlFoot.hpp"
#include "message/skill/Walk.hpp"

#include "utility/actuation/InverseKinematics.hpp"
#include "utility/input/FrameID.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::tools {

    using extension::Configuration;


    using message::actuation::KinematicsModel;
    using message::actuation::LeftLeg;
    using message::actuation::RightLeg;
    using message::actuation::ServoCommand;
    using message::actuation::ServoState;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;

    using utility::actuation::kinematics::calculate_leg_joints;
    using utility::actuation::tinyrobotics::servos_to_configuration;
    using utility::input::ServoID;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::nusight::graph;
    using utility::support::Expression;


    WalkingReferenceGenerator::WalkingReferenceGenerator(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("WalkingReferenceGenerator.yaml").then([this](const Configuration& config) {
            // Use configuration here from file WalkingReferenceGenerator.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.file_path = config["file_path"].as<std::string>();

            // Configure the motion generation options
            cfg.walk_generator_parameters.step_period     = config["walk"]["period"].as<double>();
            cfg.walk_generator_parameters.step_apex_ratio = config["walk"]["step"]["apex_ratio"].as<double>();
            cfg.walk_generator_parameters.step_limits     = config["walk"]["step"]["limits"].as<Expression>();
            cfg.walk_generator_parameters.step_height     = config["walk"]["step"]["height"].as<double>();
            cfg.walk_generator_parameters.step_width      = config["walk"]["step"]["width"].as<double>();
            cfg.walk_generator_parameters.torso_height    = config["walk"]["torso"]["height"].as<double>();
            cfg.walk_generator_parameters.torso_pitch     = config["walk"]["torso"]["pitch"].as<Expression>();
            cfg.walk_generator_parameters.torso_position_offset =
                config["walk"]["torso"]["position_offset"].as<Expression>();
            cfg.walk_generator_parameters.torso_sway_offset = config["walk"]["torso"]["sway_offset"].as<Expression>();
            cfg.walk_generator_parameters.torso_sway_ratio  = config["walk"]["torso"]["sway_ratio"].as<double>();
            cfg.walk_generator_parameters.torso_final_position_ratio =
                config["walk"]["torso"]["final_position_ratio"].as<Expression>();
            walk_generator.set_parameters(cfg.walk_generator_parameters);
            cfg.walk_generator_parameters.only_switch_when_planted =
                config["walk"]["only_switch_when_planted"].as<bool>();

            // Reset the walk engine and last update time
            walk_generator.reset();
            last_update_time = NUClear::clock::now();

            // Configure the arms
            for (auto id : utility::input::LimbID::servos_for_arms()) {
                cfg.servo_states[id] = ServoState(config["gains"]["arms"].as<double>(), 100);
            }
            cfg.arm_positions.emplace_back(ServoID::R_SHOULDER_PITCH,
                                           config["arms"]["right_shoulder_pitch"].as<double>());
            cfg.arm_positions.emplace_back(ServoID::L_SHOULDER_PITCH,
                                           config["arms"]["left_shoulder_pitch"].as<double>());
            cfg.arm_positions.emplace_back(ServoID::R_SHOULDER_ROLL,
                                           config["arms"]["right_shoulder_roll"].as<double>());
            cfg.arm_positions.emplace_back(ServoID::L_SHOULDER_ROLL, config["arms"]["left_shoulder_roll"].as<double>());
            cfg.arm_positions.emplace_back(ServoID::R_ELBOW, config["arms"]["right_elbow"].as<double>());
            cfg.arm_positions.emplace_back(ServoID::L_ELBOW, config["arms"]["left_elbow"].as<double>());

            // Configure the ranges - updated to match YAML structure
            auto x_vel_range       = config["ranges"]["x_vel_range"].as<std::vector<double>>();
            auto y_vel_range       = config["ranges"]["y_vel_range"].as<std::vector<double>>();
            auto yaw_vel_range     = config["ranges"]["yaw_vel_range"].as<std::vector<double>>();
            cfg.ranges.increment   = config["ranges"]["vel_increment"].as<double>();
            cfg.sampling_time_step = config["ranges"]["sampling_time_step"].as<double>();

            // Generate velocity vectors from ranges
            cfg.ranges.x.clear();
            cfg.ranges.y.clear();
            cfg.ranges.yaw.clear();

            for (double x = x_vel_range[0]; x <= x_vel_range[1] + 1e-6; x += cfg.ranges.increment) {
                cfg.ranges.x.push_back(x);
            }
            for (double y = y_vel_range[0]; y <= y_vel_range[1] + 1e-6; y += cfg.ranges.increment) {
                cfg.ranges.y.push_back(y);
            }
            for (double yaw = yaw_vel_range[0]; yaw <= yaw_vel_range[1] + 1e-6; yaw += cfg.ranges.increment) {
                cfg.ranges.yaw.push_back(yaw);
            }

            // Parse NUgus URDF file description
            cfg.urdf_path     = config["urdf_path"].as<std::string>();
            nugus_model_left  = tinyrobotics::import_urdf<double, n_joints>(cfg.urdf_path);
            nugus_model_right = tinyrobotics::import_urdf<double, n_joints>(cfg.urdf_path);

            // tinyrobotics IK options
            options.tolerance      = config["ik_tolerance"].as<double>();
            options.max_iterations = config["ik_max_iterations"].as<int>();
            options.method         = ik_string_to_method(config["ik_method"].as<std::string>());

            // Link names in tinyrobotics model
            cfg.torso_name      = config["links"]["torso"].as<std::string>();
            cfg.left_foot_name  = config["links"]["left_foot"].as<std::string>();
            cfg.right_foot_name = config["links"]["right_foot"].as<std::string>();
        });

        on<Startup, With<KinematicsModel>>().then([this](const KinematicsModel& kinematics_model_msg) {
            kinematics_model = kinematics_model_msg;
            log<INFO>("Generating reference walking trajectories...");

            nlohmann::json reference_data;
            reference_data["trajectories"] = nlohmann::json::array();

            // Iterate through all velocity combinations
            for (double x_vel : cfg.ranges.x) {
                for (double y_vel : cfg.ranges.y) {
                    for (double yaw_vel : cfg.ranges.yaw) {
                        // Create velocity target
                        Eigen::Vector3d velocity_target(x_vel, y_vel, yaw_vel);

                        // Generate trajectory for this velocity
                        auto trajectory = generate_trajectory(velocity_target);

                        // Add to reference data
                        nlohmann::json traj_entry;
                        traj_entry["velocity_target"] = {x_vel, y_vel, yaw_vel};
                        traj_entry["trajectory"]      = trajectory;
                        reference_data["trajectories"].push_back(traj_entry);
                    }
                }
            }

            // Write the JSON to file
            std::ofstream file(cfg.file_path);
            if (file.is_open()) {
                file << reference_data.dump(2);  // Pretty print with 2-space indentation
                file.close();
                log<INFO>("Generated reference trajectories and saved to:", cfg.file_path);
                log<INFO>("Total trajectories generated:", reference_data["trajectories"].size());
            }
            else {
                log<ERROR>("Failed to create JSON file at:", cfg.file_path);
            }
        });
    }

    nlohmann::json WalkingReferenceGenerator::generate_trajectory(const Eigen::Vector3d& velocity_target) {
        // Reset walk generator for clean start
        walk_generator.reset();

        nlohmann::json trajectory = nlohmann::json::array();

        // Simulate for one complete step cycle plus some stabilization time
        double current_time = 0.0;
        double phase_time   = 0.0;
        int steps           = 0;

        // Simulate planted foot phase (assuming starting with left foot planted)
        WalkState::Phase planted_foot_phase = WalkState::Phase::LEFT;

        while (steps < 2) {
            // Update walk generator
            walk_generator.update(cfg.sampling_time_step, velocity_target, planted_foot_phase);

            // Get foot poses in torso frame
            Eigen::Isometry3d Htl = walk_generator.get_foot_pose(utility::input::LimbID::LEFT_LEG);
            Eigen::Isometry3d Htr = walk_generator.get_foot_pose(utility::input::LimbID::RIGHT_LEG);


            // Perform IK
            auto joints_left  = calculate_leg_joints<double>(kinematics_model, Htl, LimbID::LEFT_LEG);
            auto joints_right = calculate_leg_joints<double>(kinematics_model, Htr, LimbID::RIGHT_LEG);

            auto servos_left  = std::make_unique<LeftLeg>();
            auto servos_right = std::make_unique<RightLeg>();
            for (const auto& joint : joints_left) {
                servos_left->servos[joint.first] =
                    ServoCommand(NUClear::clock::now(), joint.second, ServoState(0, 100));
            }
            for (const auto& joint : joints_right) {
                servos_right->servos[joint.first] =
                    ServoCommand(NUClear::clock::now(), joint.second, ServoState(0, 100));
            }

            // Warm start the optimisation based IK with the analytical solution
            auto q0_left  = servos_to_configuration<LeftLeg, double, 20>(servos_left.get());
            auto q0_right = servos_to_configuration<RightLeg, double, 20>(servos_right.get());

            // Run the optimisation based IK
            Eigen::Matrix<double, 20, 1> q_sol_left  = tinyrobotics::inverse_kinematics(nugus_model_left,
                                                                                       cfg.left_foot_name,
                                                                                       cfg.torso_name,
                                                                                       Htl,
                                                                                       q0_left,
                                                                                       options);
            Eigen::Matrix<double, 20, 1> q_sol_right = tinyrobotics::inverse_kinematics(nugus_model_right,
                                                                                        cfg.right_foot_name,
                                                                                        cfg.torso_name,
                                                                                        Htr,
                                                                                        q0_right,
                                                                                        options);

            // Merge the IK solutions
            Eigen::Matrix<double, 20, 1> q_sol = Eigen::Matrix<double, 20, 1>::Zero();
            for (auto servo : utility::input::LimbID::servos_for_limb(LimbID::LEFT_LEG)) {
                q_sol(servo_id_to_joint_id[servo]) = q_sol_left(servo_id_to_joint_id[servo]);
            }
            for (auto servo : utility::input::LimbID::servos_for_limb(LimbID::RIGHT_LEG)) {
                q_sol(servo_id_to_joint_id[servo]) = q_sol_right(servo_id_to_joint_id[servo]);
            }
            // Add arm joint positions from cfg.arm_positions
            for (auto servo : cfg.arm_positions) {
                q_sol(servo_id_to_joint_id[servo.first]) = servo.second;
            }

            // Create trajectory point
            nlohmann::json point;
            point["time"] = current_time;

            // Left foot pose
            point["left_foot"] = {{"position", {Htl.translation().x(), Htl.translation().y(), Htl.translation().z()}},
                                  {"orientation",
                                   {{Htl.linear()(0, 0), Htl.linear()(0, 1), Htl.linear()(0, 2)},
                                    {Htl.linear()(1, 0), Htl.linear()(1, 1), Htl.linear()(1, 2)},
                                    {Htl.linear()(2, 0), Htl.linear()(2, 1), Htl.linear()(2, 2)}}}};

            // Right foot pose
            point["right_foot"] = {{"position", {Htr.translation().x(), Htr.translation().y(), Htr.translation().z()}},
                                   {"orientation",
                                    {{Htr.linear()(0, 0), Htr.linear()(0, 1), Htr.linear()(0, 2)},
                                     {Htr.linear()(1, 0), Htr.linear()(1, 1), Htr.linear()(1, 2)},
                                     {Htr.linear()(2, 0), Htr.linear()(2, 1), Htr.linear()(2, 2)}}}};

            // Add the joint positions to the trajectory
            point["q_pos"] = std::vector<double>(q_sol.data(), q_sol.data() + q_sol.size());

            trajectory.push_back(point);

            current_time += cfg.sampling_time_step;
            phase_time += cfg.sampling_time_step;

            // Reset time and switch foot phase after one step
            if (current_time >= cfg.walk_generator_parameters.step_period) {
                steps++;
                planted_foot_phase =
                    planted_foot_phase == WalkState::Phase::LEFT ? WalkState::Phase::RIGHT : WalkState::Phase::LEFT;
                phase_time = 0.0;
            }
        }

        return trajectory;
    }

    InverseKinematicsMethod WalkingReferenceGenerator::ik_string_to_method(const std::string& method_string) {
        static std::map<std::string, InverseKinematicsMethod> string_to_method_map = {
            {"JACOBIAN", InverseKinematicsMethod::JACOBIAN},
            {"NLOPT", InverseKinematicsMethod::NLOPT},
            {"LEVENBERG_MARQUARDT", InverseKinematicsMethod::LEVENBERG_MARQUARDT},
            {"PARTICLE_SWARM", InverseKinematicsMethod::PARTICLE_SWARM},
            {"BFGS", InverseKinematicsMethod::BFGS}};

        auto it = string_to_method_map.find(method_string);
        if (it == string_to_method_map.end()) {
            throw std::invalid_argument("Unrecognized method string: " + method_string);
        }
        return it->second;
    }
}  // namespace module::tools
