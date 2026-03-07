/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
#include "FieldLocalisationNLopt.hpp"

#include <fstream>

#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/input/Sensors.hpp"

#include "utility/algorithm/assignment.hpp"
#include "utility/math/euler.hpp"

namespace module::localisation {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::localisation::FinishReset;
    using message::localisation::Line;
    using message::localisation::PenaltyReset;
    using message::localisation::ResetFieldLocalisation;
    using message::localisation::RobotPoseGroundTruth;
    using message::localisation::TeammateObservedSelf;
    using message::localisation::TeammateVisionLandmark;
    using message::localisation::UncertaintyResetFieldLocalisation;
    using message::vision::FieldLines;

    using utility::localisation::OccupancyMap;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::nusight::graph;
    using utility::support::Expression;

    FieldLocalisationNLopt::FieldLocalisationNLopt(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("FieldLocalisationNLopt.yaml").then([this](const Configuration& config) {
            this->log_level                   = config["log_level"].as<NUClear::LogLevel>();
            cfg.grid_size                     = config["grid_size"].as<double>();
            cfg.save_map                      = config["save_map"].as<bool>();
            cfg.starting_side                 = config["starting_side"].as<std::string>();
            cfg.start_time_delay              = std::chrono::seconds(config["start_time_delay"].as<int>());
            cfg.initial_state                 = Eigen::Vector3d(config["initial_state"].as<Expression>());
            cfg.use_ground_truth_localisation = config["use_ground_truth_localisation"].as<bool>();
            cfg.use_hungarian                 = config["use_hungarian"].as<bool>();
            cfg.out_of_field_cost             = config["out_of_field_cost"].as<double>();

            // Uncertainty reset parameters
            cfg.reset_on_cost             = config["reset_on_cost"].as<bool>();
            cfg.cost_threshold            = config["cost_threshold"].as<double>();
            cfg.startup_cost_threshold    = config["startup_cost_threshold"].as<double>();
            cfg.startup_teammate_wait_s   = config["startup_teammate_wait_s"].as<double>();
            cfg.reset_delay    = config["reset_delay"].as<int>();
            cfg.max_over_cost  = config["max_over_cost"].as<int>();
            cfg.step_size      = config["step_size"].as<double>();
            cfg.window_size    = config["window_size"].as<double>();
            cfg.num_angles     = config["num_angles"].as<int>();

            // Field line optimisation parameters
            cfg.field_line_distance_weight = config["field_line_distance_weight"].as<double>();
            cfg.min_field_line_points      = config["min_field_line_points"].as<int>();

            // Field line intersection optimisation parameters
            cfg.field_line_intersection_weight = config["field_line_intersection_weight"].as<double>();
            cfg.min_field_line_intersections   = config["min_field_line_intersections"].as<int>();
            cfg.max_association_distance       = config["max_association_distance"].as<double>();

            // Constraints and weighting on change in state
            cfg.change_limit        = Eigen::Vector3d(config["change_limit"].as<Expression>());
            cfg.state_change_weight = config["state_change_weight"].as<double>();

            cfg.goal_post_distance_weight = config["goal_post_distance_weight"].as<double>();
            cfg.goal_post_error_tolerance = config["goal_post_error_tolerance"].as<double>();

            // Optimisation parameters
            cfg.xtol_rel = config["opt"]["xtol_rel"].as<double>();
            cfg.ftol_rel = config["opt"]["ftol_rel"].as<double>();
            cfg.maxeval  = config["opt"]["maxeval"].as<int>();

            // Exponential filter parameters
            cfg.alpha = Eigen::Vector3d(config["exponential_filter"]["alpha"].as<Expression>());

            // Team-assisted localisation parameters
            cfg.teammate_position_weight  = config["teammate_position_weight"].as<double>();
            cfg.teammate_position_timeout = config["teammate_position_timeout"].as<double>();
            cfg.teammate_landmark_weight  = config["teammate_landmark_weight"].as<double>();
            cfg.teammate_landmark_timeout = config["teammate_landmark_timeout"].as<double>();
        });

        on<Startup, Trigger<FieldDescription>>().then("Update Field Line Map", [this](const FieldDescription& fd) {
            // Generate the field line distance map
            fieldline_distance_map = utility::localisation::setup_fieldline_distance_map(fd, cfg.grid_size);
            // Generate the field landmarks
            landmarks = utility::localisation::setup_field_landmarks(fd);
            if (cfg.save_map) {
                std::ofstream file("recordings/fieldline_map.csv");
                file << fieldline_distance_map.get_map();
                file.close();
            }
            // Generate goal posts
            Eigen::Vector3d own_left_goal =
                Eigen::Vector3d(fd.dimensions.field_length / 2, fd.dimensions.goal_width / 2, 0);
            Eigen::Vector3d own_right_goal =
                Eigen::Vector3d(fd.dimensions.field_length / 2, -fd.dimensions.goal_width / 2, 0);
            Eigen::Vector3d opp_left_goal =
                Eigen::Vector3d(-fd.dimensions.field_length / 2, fd.dimensions.goal_width / 2, 0);
            Eigen::Vector3d opp_right_goal =
                Eigen::Vector3d(-fd.dimensions.field_length / 2, -fd.dimensions.goal_width / 2, 0);
            own_goal_posts.left  = own_left_goal;
            own_goal_posts.right = own_right_goal;
            opp_goal_posts.left  = opp_left_goal;
            opp_goal_posts.right = opp_right_goal;

            // Calculate the expected distance between the goal posts
            expected_goal_post_distance = (own_left_goal - own_right_goal).norm();

            // Set the initial state as either left, right, both sides of the field or manually specified initial state
            auto left_middle_side =
                Eigen::Vector3d((2 * fd.dimensions.field_length / 8), (fd.dimensions.field_width / 2), -M_PI_2);
            auto left_bottom_side =
                Eigen::Vector3d((1 * fd.dimensions.field_length / 8), (fd.dimensions.field_width / 2), -M_PI_2);
            auto left_top_side =
                Eigen::Vector3d((3 * fd.dimensions.field_length / 8), (fd.dimensions.field_width / 2), -M_PI_2);
            auto right_middle_side =
                Eigen::Vector3d((2 * fd.dimensions.field_length / 8), (-fd.dimensions.field_width / 2), M_PI_2);
            auto right_bottom_side =
                Eigen::Vector3d((1 * fd.dimensions.field_length / 8), (-fd.dimensions.field_width / 2), M_PI_2);
            auto right_top_side =
                Eigen::Vector3d((3 * fd.dimensions.field_length / 8), (-fd.dimensions.field_width / 2), M_PI_2);
            switch (cfg.starting_side) {
                case StartingSide::LEFT: cfg.initial_hypotheses.emplace_back(left_middle_side); break;
                case StartingSide::RIGHT: cfg.initial_hypotheses.emplace_back(right_middle_side); break;
                case StartingSide::EITHER:
                    cfg.initial_hypotheses.emplace_back(left_middle_side);
                    cfg.initial_hypotheses.emplace_back(left_bottom_side);
                    cfg.initial_hypotheses.emplace_back(left_top_side);
                    cfg.initial_hypotheses.emplace_back(right_middle_side);
                    cfg.initial_hypotheses.emplace_back(right_bottom_side);
                    cfg.initial_hypotheses.emplace_back(right_top_side);
                    break;
                case StartingSide::CUSTOM: cfg.initial_hypotheses.emplace_back(cfg.initial_state); break;
                default: log<ERROR>("Invalid starting_side specified"); break;
            }
            state = cfg.initial_hypotheses[0];
            emit<Scope::DELAY>(std::make_unique<ResetFieldLocalisation>(), cfg.start_time_delay);
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
        });

        on<Trigger<ResetFieldLocalisation>>().then([this] {
            log<INFO>("Resetting field localisation");
            state               = cfg.initial_hypotheses[0];
            filtered_state      = state;
            first_measurement   = true;
            startup             = true;
            startup_start_time  = NUClear::clock::now();
            startup_best_cost   = std::numeric_limits<double>::max();
            startup_best_state  = Eigen::Vector3d::Zero();
            last_reset          = NUClear::clock::now();
        });

        on<Trigger<PenaltyReset>>().then([this](const PenaltyReset& reset) {
            log<INFO>("Resetting field localisation for penalty kick");
            state              = reset.penalty_kick_position;
            filtered_state     = state;
            first_measurement  = true;
            last_reset         = NUClear::clock::now();
            last_certain_state = state;  // Update the last certain state
        });

        on<Trigger<TeammateVisionLandmark>>().then([this](const TeammateVisionLandmark& lm) {
            Eigen::Isometry3d Hwc = Eigen::Isometry3d(lm.Hcw).inverse();
            TeammateVisionLandmarkStored stored;
            stored.rRWw = Hwc * lm.rRCc.cast<double>();
            stored.rRFf = lm.rRFf.cast<double>();
            stored.time = NUClear::clock::now();
            teammate_landmarks.push_back(stored);
            // Keep only the most recent 10 observations
            if (teammate_landmarks.size() > 10) {
                teammate_landmarks.erase(teammate_landmarks.begin());
            }
        });

        on<Trigger<TeammateObservedSelf>>().then([this](const TeammateObservedSelf& obs) {
            if (!std::isfinite(obs.position.x()) || !std::isfinite(obs.position.y())) {
                log<WARN>("TeammateObservedSelf: ignoring NaN/inf position from teammate");
                return;
            }
            has_teammate_obs        = true;
            teammate_obs_position_f = Eigen::Vector2d(obs.position.x(), obs.position.y());
            teammate_obs_cost       = obs.sender_cost > 0.0f ? obs.sender_cost : 1.0f;
            teammate_obs_time       = NUClear::clock::now();

            Eigen::Vector2d current_pos(filtered_state.x(), filtered_state.y());
            double disagreement = (current_pos - teammate_obs_position_f).norm();

            log<INFO>("TeammateObservedSelf: teammate reports us at (",
                      obs.position.x(),
                      ", ",
                      obs.position.y(),
                      ") sender_cost=",
                      teammate_obs_cost,
                      " | our pos=(",
                      filtered_state.x(),
                      ", ",
                      filtered_state.y(),
                      ") disagreement=",
                      disagreement,
                      "m | startup=",
                      startup,
                      " | will_use_in_cost=",
                      !startup);

            if (!startup && obs.sender_cost < 0.5f && disagreement > 3.0) {
                // Check if the teammate's report matches our mirror position (-x, -y, θ+π).
                // If so, we're on the wrong half of the field — flip directly rather than doing
                // a full global reset. This is the "choose a side" mechanism.
                Eigen::Vector2d mirror_pos(-filtered_state.x(), -filtered_state.y());
                double mirror_agreement = (mirror_pos - teammate_obs_position_f).norm();

                if (mirror_agreement < 2.0) {
                    double mirror_theta = filtered_state.z() + M_PI;
                    log<WARN>("TeammateObservedSelf: MIRROR DETECTED — teammate places us at (",
                              teammate_obs_position_f.x(),
                              ", ",
                              teammate_obs_position_f.y(),
                              ") which is ",
                              mirror_agreement,
                              "m from our mirror (",
                              mirror_pos.x(),
                              ", ",
                              mirror_pos.y(),
                              ") — flipping directly");
                    state              = Eigen::Vector3d(mirror_pos.x(), mirror_pos.y(), mirror_theta);
                    filtered_state     = state;
                    last_certain_state = state;
                    num_over_cost      = 0;
                }
                else {
                    log<WARN>("TeammateObservedSelf: STRONG DISAGREEMENT of ",
                              disagreement,
                              "m (mirror also off by ",
                              mirror_agreement,
                              "m) — forcing global reset");
                    force_global_reset = true;
                    num_over_cost      = cfg.max_over_cost + 1;
                }
            }
        });

        on<Trigger<FieldLines>,
           Optional<With<FieldIntersections>>,
           Optional<With<Goals>>,
           With<Stability>,
           Optional<With<RobotPoseGroundTruth>>,
           With<FieldDescription>,
           With<Sensors>,
           Single>()
            .then(
                "NLopt field localisation",
                [this](const FieldLines& field_lines,
                       const std::shared_ptr<const FieldIntersections>& field_intersections,
                       const std::shared_ptr<const Goals>& goals,
                       const Stability& stability,
                       const std::shared_ptr<const RobotPoseGroundTruth>& robot_pose_ground_truth,
                       const FieldDescription& fd,
                       const Sensors& sensors) {
                    // Emit field message using ground truth if available
                    if (cfg.use_ground_truth_localisation && robot_pose_ground_truth) {
                        auto field(std::make_unique<Field>());
                        // Odometry ground truth should be field {f} to torso {t} space, so we can assume the identity
                        // transform

                        if (!ground_truth_initialised) {
                            Eigen::Isometry3d Hft                    = Eigen::Isometry3d(robot_pose_ground_truth->Hft);
                            ground_truth_Hfw.translation().head<2>() = Hft.translation().head<2>();
                            ground_truth_Hfw.translation()[2]        = 0;
                            double yaw                               = mat_to_rpy_intrinsic(Hft.rotation()).z();
                            ground_truth_Hfw.linear()                = rpy_intrinsic_to_mat(Eigen::Vector3d(0, 0, yaw));
                            ground_truth_initialised                 = true;
                        }
                        field->Hfw = ground_truth_Hfw;
                        emit(field);
                        return;
                    }

                    // Don't run an update if there are not enough field line points or the robot is
                    // unstable
                    bool unstable = stability <= Stability::FALLING;
                    if (unstable || field_lines.rPWw.size() < cfg.min_field_line_points) {
                        log<DEBUG>("Not enough field line points or robot is unstable");
                        return;
                    }

                    double chosen_state_cost = 0.0;
                    Eigen::Vector3d proposed_state;

                    if (startup && cfg.starting_side == StartingSide::EITHER) {
                        // Startup search: sweep both sidelines in x at step_size intervals,
                        // using only inward-facing headings. Robots always spawn on a sideline
                        // facing the field, so no other headings need to be evaluated.
                        // Top sideline (y=+half_width) faces inward at heading = -π/2 (toward -y).
                        // Bottom sideline (y=-half_width) faces inward at heading = +π/2 (toward +y).
                        // The NLopt optimizer refines heading from these seeds within ±change_limit.z().
                        double half_length = fd.dimensions.field_length / 2.0;
                        double half_width  = fd.dimensions.field_width / 2.0;

                        double startup_elapsed =
                            std::chrono::duration_cast<std::chrono::duration<double>>(NUClear::clock::now()
                                                                                      - startup_start_time)
                                .count();

                        int num_x_steps   = static_cast<int>(2.0 * half_length / cfg.step_size) + 1;
                        int total_hyps    = num_x_steps * 2;  // top + bottom sideline

                        log<INFO>("--- Startup search frame ---");
                        log<INFO>("  Field: length=",
                                  2.0 * half_length,
                                  "m width=",
                                  2.0 * half_width,
                                  "m | step_size=",
                                  cfg.step_size,
                                  " | evaluating ",
                                  total_hyps,
                                  " hypotheses (sidelines only, inward-facing)");
                        log<INFO>("  Elapsed in startup: ",
                                  startup_elapsed,
                                  "s / teammate_wait=",
                                  cfg.startup_teammate_wait_s,
                                  "s | has_teammate_obs=",
                                  has_teammate_obs,
                                  " | field_line_pts=",
                                  field_lines.rPWw.size());

                        std::vector<std::pair<Eigen::Vector3d, double>> opt_results{};
                        for (double x = -half_length; x <= half_length; x += cfg.step_size) {
                            // Top sideline: face inward (toward -y = heading -π/2)
                            opt_results.push_back(run_field_line_optimisation(
                                Eigen::Vector3d(x, half_width, -M_PI_2),
                                field_lines.rPWw,
                                field_intersections,
                                goals));
                            // Bottom sideline: face inward (toward +y = heading +π/2)
                            opt_results.push_back(run_field_line_optimisation(
                                Eigen::Vector3d(x, -half_width, M_PI_2),
                                field_lines.rPWw,
                                field_intersections,
                                goals));
                        }

                        auto best_hypothesis =
                            std::min_element(opt_results.begin(), opt_results.end(), [](const auto& a, const auto& b) {
                                return a.second < b.second;
                            });
                        proposed_state    = best_hypothesis->first;
                        chosen_state_cost = best_hypothesis->second;

                        // Count how many hypotheses are below the commit threshold (gives confidence signal)
                        int good_hyp_count =
                            static_cast<int>(std::count_if(opt_results.begin(),
                                                           opt_results.end(),
                                                           [this](const auto& h) {
                                                               return h.second < cfg.startup_cost_threshold;
                                                           }));

                        log<INFO>("  Best hypothesis: cost=",
                                  chosen_state_cost,
                                  " at (x=",
                                  proposed_state.x(),
                                  " y=",
                                  proposed_state.y(),
                                  " θ=",
                                  proposed_state.z(),
                                  "rad) | hypotheses below threshold: ",
                                  good_hyp_count,
                                  "/",
                                  total_hyps);

                        // Track the best hypothesis seen across ALL startup frames.
                        // At timeout we commit to this rather than whatever the current frame shows,
                        // since the timeout frame may coincidentally have worse field line visibility.
                        if (chosen_state_cost < startup_best_cost) {
                            startup_best_cost  = chosen_state_cost;
                            startup_best_state = proposed_state;
                            log<INFO>("  New all-time best: cost=",
                                      startup_best_cost,
                                      " at (x=",
                                      startup_best_state.x(),
                                      " y=",
                                      startup_best_state.y(),
                                      " θ=",
                                      startup_best_state.z(),
                                      "rad)");
                        }

                        // Always update state to the best candidate so the emitted field position is
                        // meaningful (not zero). Broadcasted with cost=999 so teammates ignore it.
                        state          = proposed_state;
                        filtered_state = proposed_state;

                        // Commit once confident: cost is low AND either a teammate has confirmed
                        // our position (breaking mirror ambiguity) or we've waited long enough.
                        bool teammate_confirmed = has_teammate_obs;
                        bool timed_out          = startup_elapsed > cfg.startup_teammate_wait_s;
                        bool cost_ok            = chosen_state_cost < cfg.startup_cost_threshold;

                        if (cost_ok && (teammate_confirmed || timed_out)) {
                            // At timeout, use the all-time best — not the current (possibly poor) frame.
                            Eigen::Vector3d commit_state =
                                timed_out ? startup_best_state : proposed_state;
                            double commit_cost = timed_out ? startup_best_cost : chosen_state_cost;

                            state          = commit_state;
                            filtered_state = commit_state;

                            log<INFO>("  COMMITTING: cost=",
                                      commit_cost,
                                      " at (x=",
                                      state.x(),
                                      " y=",
                                      state.y(),
                                      " θ=",
                                      state.z(),
                                      "rad) | reason: ",
                                      teammate_confirmed ? "teammate_confirmed" : "timed_out",
                                      " | all-time best cost=",
                                      startup_best_cost);
                            last_certain_state = state;
                            first_measurement  = true;
                            startup            = false;
                        }
                        else {
                            log<INFO>("  Not committing: cost_ok=",
                                      cost_ok,
                                      " teammate_confirmed=",
                                      teammate_confirmed,
                                      " timed_out=",
                                      timed_out,
                                      " | all-time best so far: cost=",
                                      startup_best_cost,
                                      " | waiting for ",
                                      teammate_confirmed ? "lower cost" : "teammate or timeout");
                        }
                    }
                    else {
                        // Run the optimisation routine
                        std::pair<Eigen::Vector3d, double> opt_results =
                            run_field_line_optimisation(filtered_state, field_lines.rPWw, field_intersections, goals);
                        proposed_state    = opt_results.first;
                        chosen_state_cost = opt_results.second;

                        // Only accept the optimisation result if the cost is below the threshold
                        if (chosen_state_cost < cfg.cost_threshold) {
                            state = proposed_state;

                            // Apply exponential filter to smooth the state estimate unless it is the first measurement.
                            filtered_state =
                                first_measurement
                                    ? state
                                    : cfg.alpha.cwiseProduct(state)
                                          + (Eigen::Vector3d::Ones() - cfg.alpha).cwiseProduct(filtered_state);

                            // Reset the flag.
                            first_measurement = false;

                            // Update the last certain state and reset counter
                            last_certain_state = filtered_state;
                            num_over_cost      = 0;
                        }
                        else {
                            // Reject the update, keep previous filtered state
                            log<DEBUG>("Rejecting optimisation result: cost ",
                                       chosen_state_cost,
                                       " exceeds threshold ",
                                       cfg.cost_threshold);
                            // Keep the current filtered_state unchanged
                            // Increment the over-cost counter for potential reset
                            num_over_cost++;
                        }
                    }

                    // Check if uncertainty is too high and trigger reset if needed
                    emit(graph("Cost", chosen_state_cost));
                    if (cfg.reset_on_cost && (num_over_cost > cfg.max_over_cost)
                        && ((NUClear::clock::now() - last_reset) > std::chrono::seconds(cfg.reset_delay))) {
                        // Cost has been high too many times, reset the localisation
                        log<WARN>("Cost exceeded threshold ",
                                  cfg.max_over_cost,
                                  " times, triggering uncertainty reset");
                        // Emit that we are resetting, eg for behaviour
                        emit(std::make_unique<UncertaintyResetFieldLocalisation>());
                        // Reset localisation by finding a new low cost state
                        uncertainty_reset(fd, field_lines, field_intersections, goals, sensors.Hrw);
                        // Reset variables
                        num_over_cost = 0;
                        last_reset    = NUClear::clock::now();
                        // Let other modules know that localisation has finished resetting
                        emit<Scope::DELAY>(std::make_unique<FinishReset>(), std::chrono::seconds(1));
                    }

                    // Emit the field message
                    auto field = std::make_unique<Field>();
                    field->Hfw = compute_Hfw(filtered_state);

                    // Debugging
                    if (log_level <= DEBUG) {
                        debug_field_localisation(field->Hfw);
                    }
                    // Association (run once for debugging in NUsight)
                    auto associations = data_association(field_intersections, field->Hfw);
                    for (const auto& association : associations) {
                        field->association_lines.push_back({association.first, association.second});
                    }

                    // During startup, broadcast a very high cost so teammates know not to trust
                    // this robot's position yet. This prevents stale startup positions from being
                    // relayed back via TeammateObservedSelf and corrupting subsequent startup attempts.
                    field->cost = startup ? 999.0 : chosen_state_cost;

                    emit(field);
                });
    }

    void FieldLocalisationNLopt::debug_field_localisation(Eigen::Isometry3d Hfw) {
        emit(graph("opt state", state.x(), state.y(), state.z()));
        emit(graph("filtered state", filtered_state.x(), filtered_state.y(), filtered_state.z()));
        // Determine translational distance error
        Eigen::Vector3d true_rFWw  = ground_truth_Hfw.translation();
        Eigen::Vector3d rFWw       = (Hfw.translation());
        Eigen::Vector3d error_rFWw = (true_rFWw - rFWw).cwiseAbs();
        // Determine yaw, pitch, and roll error
        Eigen::Vector3d true_Rfw  = mat_to_rpy_intrinsic(ground_truth_Hfw.rotation());
        Eigen::Vector3d Rfw       = mat_to_rpy_intrinsic(Hfw.rotation());
        Eigen::Vector3d error_Rfw = (true_Rfw - Rfw).cwiseAbs();
        double quat_rot_error     = Eigen::Quaterniond(ground_truth_Hfw.linear() * Hfw.inverse().linear()).w();

        // Graph translation and error from ground truth
        emit(graph("Hfw true translation (rFWw)", true_rFWw.x(), true_rFWw.y(), true_rFWw.z()));
        emit(graph("Hfw translation error", error_rFWw.x(), error_rFWw.y(), error_rFWw.z()));
        // Graph rotation and error from ground truth
        emit(graph("Rfw true angles (rpy)", true_Rfw.x(), true_Rfw.y(), true_Rfw.z()));
        emit(graph("Rfw error (rpy)", error_Rfw.x(), error_Rfw.y(), error_Rfw.z()));
        emit(graph("Rfw quaternion rotational error", quat_rot_error));
    }

    Eigen::Isometry3d FieldLocalisationNLopt::compute_Hfw(const Eigen::Vector3d& state) {
        return Eigen::Translation<double, 3>(state.x(), state.y(), 0)
               * Eigen::AngleAxis<double>(state.z(), Eigen::Matrix<double, 3, 1>::UnitZ());
    }

    Eigen::Vector2i FieldLocalisationNLopt::position_in_map(const Eigen::Vector3d& particle,
                                                            const Eigen::Vector3d& rPWw) {
        // Transform observations from world {w} to field {f} space
        Eigen::Vector3d rPFf = compute_Hfw(particle) * rPWw;

        // Get the associated index in the field line map [x, y]
        // Note: field space is placed in centre of the field, whereas the  map origin (x,y) is top left corner
        return Eigen::Vector2i(fieldline_distance_map.get_length() / 2 - std::round(rPFf(1) / cfg.grid_size),
                               fieldline_distance_map.get_width() / 2 + std::round(rPFf(0) / cfg.grid_size));
    }

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> FieldLocalisationNLopt::data_association(
        const std::shared_ptr<const FieldIntersections>& field_intersections,
        const Eigen::Isometry3d& Hfw) {
        // If there are no field intersections, return an empty vector
        if (!field_intersections || field_intersections->intersections.empty()) {
            return {};
        }

        // Field intersection measurement associated with a landmark (known landmark, intersection detection)
        return cfg.use_hungarian ? hungarian_association(field_intersections, Hfw)
                                 : greedy_association(field_intersections, Hfw);
    }

    std::pair<Eigen::Vector3d, double> FieldLocalisationNLopt::run_field_line_optimisation(
        const Eigen::Vector3d& initial_guess,
        const std::vector<Eigen::Vector3d>& field_lines,
        const std::shared_ptr<const FieldIntersections>& field_intersections,
        const std::shared_ptr<const Goals>& goals) {
        // Wrap the objective function in a lambda function
        ObjectiveFunction<double, 3> obj_fun =
            [&](const Eigen::Matrix<double, 3, 1>& x, Eigen::Matrix<double, 3, 1>& grad, void* data) -> double {
            (void) data;  // Unused in this case
            (void) grad;  // Unused in this case

            // --- Field line point cost ---
            double cost = 0.0;
            for (auto rORr : field_lines) {
                // Get the position [x, y] of the observation in the map for this particle
                Eigen::Vector2i map_position = position_in_map(x, rORr);
                double occupancy_value = fieldline_distance_map.get_occupancy_value(map_position.x(), map_position.y());
                occupancy_value =
                    occupancy_value == -1 ? cfg.out_of_field_cost : occupancy_value;  // If no value, set to 3.0
                cost += cfg.field_line_distance_weight * std::pow(occupancy_value, 2);
            }
            // Average the cost by the number of field lines
            cost /= field_lines.size() > 0 ? field_lines.size() : 1;

            // Compute the cost and gradient
            auto Hfw = compute_Hfw(x);

            // --- Field line intersection cost ---
            if (field_intersections) {
                auto associations = data_association(field_intersections, Hfw);
                for (const auto& association : associations) {
                    // Calculate the distance between the observed intersection and the closest landmark
                    double distance = (association.first - association.second).norm();
                    cost += cfg.field_line_intersection_weight * std::pow(distance, 2);
                }
                // Average the cost by the number of associations
                cost /= associations.size() > 0 ? associations.size() : 1;
            }

            // --- Goal post cost ---
            // Only consider goal post cost if there are two goals
            if (goals && goals->goals.size() == 2) {
                // Ensure the goal posts are roughly correct distance apart
                auto Hwc              = Eigen::Isometry3d(goals->Hcw).inverse();
                auto rGWw_1           = Hwc * (goals->goals[0].post.bottom * goals->goals[0].post.distance);
                auto rGWw_2           = Hwc * (goals->goals[1].post.bottom * goals->goals[1].post.distance);
                double distance_apart = (rGWw_1 - rGWw_2).norm();
                if (std::abs(distance_apart - expected_goal_post_distance) < cfg.goal_post_error_tolerance) {

                    auto rGFf_left  = Hfw * rGWw_1;
                    auto rGFf_right = Hfw * rGWw_2;
                    if (rGFf_left.y() < rGFf_right.y()) {
                        std::swap(rGFf_left, rGFf_right);
                    }
                    auto expected_goal_post_postions = rGFf_left.x() > 0 ? own_goal_posts : opp_goal_posts;

                    // Calculate the distance between the goal posts
                    double left_distance  = (rGFf_left - expected_goal_post_postions.left).norm();
                    double right_distance = (rGFf_right - expected_goal_post_postions.right).norm();

                    // Add the cost of the distance between the goal posts
                    cost += cfg.goal_post_distance_weight * std::pow(left_distance, 2);
                    cost += cfg.goal_post_distance_weight * std::pow(right_distance, 2);
                }
            }

            // --- Teammate-observed position cost ---
            // A well-localised teammate has reported where they see us on the field.
            // Weight is inversely proportional to the sender's localisation cost.
            // Skipped during startup: stale observations from a wrong pre-reset broadcast would
            // corrupt the clean field-line-only startup evaluation.
            if (has_teammate_obs && !startup && cfg.teammate_position_weight > 0.0) {
                double age = std::chrono::duration_cast<std::chrono::duration<double>>(
                                 NUClear::clock::now() - teammate_obs_time)
                                 .count();
                if (age < cfg.teammate_position_timeout) {
                    Eigen::Vector2d rTFf_estimated(x[0], x[1]);
                    double weight = cfg.teammate_position_weight / (1.0 + teammate_obs_cost);
                    cost += weight * (rTFf_estimated - teammate_obs_position_f).squaredNorm();
                }
            }

            // --- Robot-as-landmark cost (only when localisation is uncertain) ---
            // When we see a teammate visually and know their field position from their broadcast,
            // the discrepancy between where our candidate Hfw places them vs their known position
            // directly constrains our own field transform. Only active when num_over_cost > 0.
            if (cfg.teammate_landmark_weight > 0.0 && !teammate_landmarks.empty()) {
                auto Hfw_candidate = compute_Hfw(x);
                for (const auto& lm : teammate_landmarks) {
                    double age = std::chrono::duration_cast<std::chrono::duration<double>>(
                                     NUClear::clock::now() - lm.time)
                                     .count();
                    if (age < cfg.teammate_landmark_timeout) {
                        Eigen::Vector3d rRFf_estimated = Hfw_candidate * lm.rRWw;
                        cost += cfg.teammate_landmark_weight
                                * (rRFf_estimated.head<2>() - lm.rRFf.head<2>()).squaredNorm();
                    }
                }
            }

            // --- State change cost ---
            cost += cfg.state_change_weight * (x - initial_guess).squaredNorm();

            return cost;
        };
        // Create the NLopt optimizer and setup the algorithm, tolerances and maximum number of evaluations
        constexpr unsigned int n   = 3;
        nlopt::algorithm algorithm = nlopt::LN_COBYLA;
        nlopt::opt opt             = nlopt::opt(algorithm, n);
        opt.set_xtol_rel(cfg.xtol_rel);
        opt.set_ftol_rel(cfg.ftol_rel);
        opt.set_maxeval(cfg.maxeval);

        // Set the objective function
        opt.set_min_objective(eigen_objective_wrapper<double, n>, &obj_fun);

        // Define the upper and lower bounds for the change in state
        Eigen::Vector3d lower_bounds = initial_guess - cfg.change_limit;
        Eigen::Vector3d upper_bounds = initial_guess + cfg.change_limit;

        // Convert bounds to std::vector for NLopt
        std::vector<double> lb(n), ub(n);
        eigen_to_nlopt<double, n>(lower_bounds, lb);
        eigen_to_nlopt<double, n>(upper_bounds, ub);

        // Set bounds in the optimizer
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);

        // Convert the initial guess to NLopt format
        std::vector<double> x(n);
        eigen_to_nlopt<double, n>(initial_guess, x);

        // Find the optimal solution
        double final_cost;
        opt.optimize(x, final_cost);

        // Convert the optimized solution back to an Eigen vector
        Eigen::Matrix<double, n, 1> optimized_solution(n);
        nlopt_to_eigen<double, n>(x, optimized_solution);
        return std::make_pair(optimized_solution, final_cost);
    }
}  // namespace module::localisation
