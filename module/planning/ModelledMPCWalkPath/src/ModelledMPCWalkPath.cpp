/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "ModelledMPCWalkPath.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Robot.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/skill/Walk.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::planning {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::input::Sensors;
    using message::localisation::Robots;
    using message::planning::WalkTo;
    using message::planning::WalkToDebug;
    using message::skill::Walk;

    using modelled_walk_mpc::HammersteinModel;
    using modelled_walk_mpc::ModelledWalkMPC;
    using utility::math::angle::vector_to_bearing;
    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::nusight::graph;
    using utility::support::Expression;

    ModelledMPCWalkPath::ModelledMPCWalkPath(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration, Sync<ModelledMPCWalkPath>>("ModelledMPCWalkPath.yaml")
            .then([this](const Configuration& config) {
                this->log_level = config["log_level"].as<NUClear::LogLevel>();

                // Limits
                cfg.mpc.max_velocity          = config["max_velocity"].as<Expression>();
                cfg.mpc.max_backward_velocity = config["max_backward_velocity"].as<double>();
                cfg.mpc.max_acceleration      = config["max_acceleration"].as<Expression>();

                // Cost
                cfg.mpc.w_position     = config["w_position"].as<double>();
                cfg.mpc.huber_delta    = config["huber_delta"].as<double>();
                cfg.mpc.w_heading      = config["w_heading"].as<double>();
                cfg.mpc.heading_radius = config["heading_radius"].as<double>();
                cfg.mpc.w_face         = config["w_face"].as<double>();
                cfg.mpc.w_effort       = config["w_effort"].as<Expression>();
                cfg.mpc.w_rate         = config["w_rate"].as<Expression>();

                // Obstacles
                cfg.mpc.obstacle_radius = config["obstacle_radius"].as<double>();
                cfg.mpc.w_slack         = config["w_slack"].as<double>();

                // The capability envelope
                cfg.mpc.envelope   = modelled_walk_mpc::envelope_from_yaml(config["envelope"]);
                cfg.mpc.w_envelope = config["w_envelope"].as<double>();

                // The policy's response
                cfg.mpc.model          = HammersteinModel::from_yaml(config["model"]);
                cfg.mpc.kink_smoothing = config["kink_smoothing"].as<Expression>();

                // Solver
                cfg.mpc.max_iterations = config["max_iterations"].as<int>();
                cfg.time_budget        = config["time_budget"].as<double>();
                cfg.stale_time         = config["stale_time"].as<double>();

                // Fallback (PlanWalkPath's WalkTo control law and dead zone), with the MPC's velocity limits
                cfg.fallback.max_velocity          = cfg.mpc.max_velocity;
                cfg.fallback.max_backward_velocity = cfg.mpc.max_backward_velocity;
                cfg.fallback.k_translation         = config["fallback"]["k_translation"].as<double>();
                cfg.fallback.k_theta               = config["fallback"]["k_theta"].as<double>();
                cfg.fallback.max_align_radius      = config["fallback"]["max_align_radius"].as<double>();
                cfg.fallback.min_align_radius      = config["fallback"]["min_align_radius"].as<double>();
                cfg.fallback.max_angle_error       = config["fallback"]["max_angle_error"].as<Expression>();
                cfg.fallback.min_angle_error       = config["fallback"]["min_angle_error"].as<Expression>();
                cfg.fallback_min_velocity          = config["fallback"]["min_velocity"].as<Expression>();
                cfg.fallback_zero_tolerance        = config["fallback"]["zero_tolerance"].as<Expression>();
                cfg.fallback_rotate_velocity_x     = config["fallback"]["rotate_velocity_x"].as<double>();

                if (mpc == nullptr) {
                    mpc = std::make_unique<ModelledWalkMPC>(cfg.mpc);
                }
                else {
                    mpc->configure(cfg.mpc);
                }
            });

        on<Trigger<Stability>, Sync<ModelledMPCWalkPath>>().then([this](const Stability& new_stability) {
            // Starting to walk after a fall or from standing: the robot is still, whatever was commanded before
            if (mpc != nullptr && (stability == Stability::FALLEN || stability == Stability::STANDING)
                && new_stability == Stability::DYNAMIC) {
                mpc->reset();
                log<DEBUG>("Resetting the MPC");
            }
            stability = new_stability;
        });

        on<Start<WalkTo>, Sync<ModelledMPCWalkPath>>().then([this] {
            // A new WalkTo: whatever was walking before, the MPC's warm start and estimate are from another request
            if (mpc != nullptr) {
                mpc->reset();
            }
        });

        on<Provide<WalkTo>, Optional<With<Robots>>, With<Sensors>, Sync<ModelledMPCWalkPath>>().then(
            [this](const WalkTo& walk_to, const std::shared_ptr<const Robots>& robots, const Sensors& sensors) {
                if (mpc == nullptr) {
                    return;
                }

                const auto now     = NUClear::clock::now();
                const double since = std::chrono::duration<double>(now - last_plan_time).count();

                // The MPC steps in ModelledWalkMPC::DT, and its acceleration limits assume one solve per step. A
                // request well before the next step resends the last command.
                if (since < 0.5 * ModelledWalkMPC::DT) {
                    emit<Task>(std::make_unique<Walk>(last_walk_command));
                    return;
                }
                // After a gap the previous command and the estimate are stale, so start from standing still.
                // Otherwise the robot has been walking with the last command since the last solve.
                if (since > cfg.stale_time) {
                    mpc->reset();
                }
                else {
                    mpc->advance(since);
                }
                last_plan_time = now;

                // Target pose in the robot frame
                const auto& Hrd                     = walk_to.Hrd;
                const Eigen::Vector2d rDRr          = Hrd.translation().head<2>();
                const double angle_to_final_heading = vector_to_bearing(Hrd.linear().col(0).head<2>());
                const Eigen::Vector3d target(rDRr.x(), rDRr.y(), angle_to_final_heading);

                // Other robots are the obstacles, in the robot frame
                std::vector<Eigen::Vector2d> obstacles{};
                if (robots) {
                    for (const auto& robot : robots->robots) {
                        obstacles.emplace_back((sensors.Hrw * robot.rRWw).head<2>());
                    }
                }

                // A solve that runs over the budget has already moved the MPC's previous command on, so keep the
                // one actually sent for the fallback's acceleration limits
                const Eigen::Vector3d u_sent               = mpc->previous_command();
                const Eigen::Vector3d delivered            = mpc->delivered_velocity();
                const modelled_walk_mpc::Solution solution = mpc->solve(target, obstacles);

                Eigen::Vector3d command = solution.command;
                const bool use_fallback = !solution.success || solution.solve_time > cfg.time_budget;
                if (use_fallback) {
                    // PlanWalkPath's control law, held to the MPC's acceleration limits so the switch is smooth, then
                    // PlanWalkPath's dead zone, as the walk stands still for commands too small to start stepping
                    const auto result   = walk_path::walk_to_velocity(rDRr, angle_to_final_heading, cfg.fallback);
                    const auto velocity = walk_path::constrain_velocity(result.velocity,
                                                                        cfg.fallback.max_velocity,
                                                                        cfg.fallback.max_backward_velocity);
                    const Eigen::Vector3d a_max = cfg.mpc.max_acceleration * ModelledWalkMPC::DT;
                    command = mpc->clip_to_limits(velocity.cwiseMax(u_sent - a_max).cwiseMin(u_sent + a_max));
                    command = mpc->clip_to_limits(walk_path::apply_dead_zone(command,
                                                                             cfg.fallback_min_velocity,
                                                                             cfg.fallback_zero_tolerance,
                                                                             cfg.mpc.max_velocity,
                                                                             cfg.fallback_rotate_velocity_x));
                    if (!command.allFinite()) {
                        command.setZero();
                    }
                    // The model's estimate runs on what was actually sent
                    mpc->set_previous_command(command);
                    ++fallbacks;
                    log<DEBUG>("MPC fell back to PlanWalkPath's control law:",
                               solution.status_string(),
                               "in",
                               solution.solve_time * 1e3,
                               "ms");
                }

                // The model accounts for the policy's dead zones, so the command is sent as planned
                last_walk_command = command;
                emit<Task>(std::make_unique<Walk>(command));

                // Visualise the plan in NUsight / PlotJuggler
                emit(graph("Walk Command", command.x(), command.y(), command.z()));
                emit(graph("MPC Delivered Velocity", delivered.x(), delivered.y(), delivered.z()));
                emit(graph("MPC Solve Time (ms)", solution.solve_time * 1e3));
                emit(graph("MPC Iterations", solution.iterations));
                emit(graph("MPC Fallback", use_fallback ? 1.0 : 0.0));
                emit(graph("MPC Envelope Violation", mpc->envelope_violation(command)));
                if (!solution.states.empty()) {
                    const Eigen::Vector3d& end = solution.states.back();
                    emit(graph("MPC Horizon End", end.x(), end.y(), end.z()));
                }

                // The same debug message as PlanWalkPath (RobotCommunication sends the walk target from it)
                auto debug                       = std::make_unique<WalkToDebug>();
                debug->Hrd.translation().head(2) = rDRr;
                debug->Hrd.linear()           = rpy_intrinsic_to_mat(Eigen::Vector3d(0.0, 0.0, angle_to_final_heading));
                debug->angle_to_target        = vector_to_bearing(rDRr);
                debug->angle_to_final_heading = angle_to_final_heading;
                debug->translational_error    = rDRr.norm();
                debug->velocity_target        = command;
                emit(debug);
            });

        on<Every<10, std::chrono::seconds>, Sync<ModelledMPCWalkPath>>().then([this] {
            if (fallbacks > 0) {
                log<WARN>("MPC fell back to PlanWalkPath's control law", fallbacks, "times in the last 10 s");
                fallbacks = 0;
            }
        });
    }

}  // namespace module::planning
