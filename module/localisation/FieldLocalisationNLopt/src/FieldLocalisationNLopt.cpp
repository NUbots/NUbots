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

#include <algorithm>
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

            // Uncertainty reset parameters (grid-search fallback)
            cfg.reset_delay = config["reset_delay"].as<int>();
            cfg.step_size   = config["step_size"].as<double>();
            cfg.window_size = config["window_size"].as<double>();
            cfg.num_angles  = config["num_angles"].as<int>();

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
            cfg.normal_xtol_rel        = config["normal_opt"]["xtol_rel"].as<double>();
            cfg.normal_ftol_rel        = config["normal_opt"]["ftol_rel"].as<double>();
            cfg.normal_maxeval         = config["normal_opt"]["maxeval"].as<int>();
            std::string normal_opt_alg = config["normal_opt"]["algorithm"].as<std::string>();
            if (auto it = nlopt_algorithm_map.find(normal_opt_alg); it != nlopt_algorithm_map.end()) {
                cfg.normal_algorithm = it->second;
            }
            else {
                log<WARN>("Unknown normal optimisation algorithm '",
                          config["normal_opt"]["algorithm"].as<std::string>(),
                          "', using default");
                cfg.normal_algorithm = nlopt::LN_COBYLA;
            }


            cfg.uncertainty_xtol_rel        = config["uncertainty_opt"]["xtol_rel"].as<double>();
            cfg.uncertainty_ftol_rel        = config["uncertainty_opt"]["ftol_rel"].as<double>();
            cfg.uncertainty_maxeval         = config["uncertainty_opt"]["maxeval"].as<int>();
            std::string uncertainty_opt_alg = config["uncertainty_opt"]["algorithm"].as<std::string>();
            if (auto it = nlopt_algorithm_map.find(uncertainty_opt_alg); it != nlopt_algorithm_map.end()) {
                cfg.uncertainty_algorithm = it->second;
            }
            else {
                log<WARN>("Unknown uncertainty optimisation algorithm '",
                          config["uncertainty_opt"]["algorithm"].as<std::string>(),
                          "', using default");
            }

            // Kalman filter tuning
            cfg.kalman_process_noise     = Eigen::Vector3d(config["kalman"]["process_noise"].as<Expression>());
            cfg.kalman_measurement_scale = config["kalman"]["measurement_scale"].as<double>();
            cfg.kalman_hessian_step      = Eigen::Vector3d(config["kalman"]["hessian_step"].as<Expression>());

            // Validity-gated acceptance
            cfg.validity_line_inlier_distance = config["validity"]["line_inlier_distance"].as<double>();
            cfg.validity_min_validity         = config["validity"]["min_validity"].as<double>();
            cfg.validity_max_invalid_frames   = config["validity"]["max_invalid_frames"].as<int>();
            cfg.validity_reset_on_invalid     = config["validity"]["reset_on_invalid"].as<bool>();

            // Reset candidate generation
            cfg.reset_use_candidates            = config["reset"]["use_candidates"].as<bool>();
            cfg.reset_pair_separation_tolerance = config["reset"]["pair_separation_tolerance"].as<double>();
            cfg.reset_max_candidates            = config["reset"]["max_candidates"].as<int>();
            cfg.reset_constrain_to_half         = config["reset"]["constrain_to_half"].as<bool>();

            // Mirror hygiene
            cfg.mirror_auto_flip = config["mirror"]["auto_flip"].as<bool>();

            // Automatic local-minimum escape
            cfg.recovery_enabled            = config["recovery"]["enabled"].as<bool>();
            cfg.recovery_probe_period       = config["recovery"]["probe_period"].as<int>();
            cfg.recovery_improvement_margin = config["recovery"]["improvement_margin"].as<double>();
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
            filter.reset(state, cfg.change_limit);
            emit<Scope::DELAY>(std::make_unique<ResetFieldLocalisation>(), cfg.start_time_delay);
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
        });

        on<Trigger<ResetFieldLocalisation>>().then([this] {
            log<INFO>("Resetting field localisation");
            state = cfg.initial_hypotheses[0];
            filter.reset(state, cfg.change_limit);
            startup        = true;
            last_reset     = NUClear::clock::now();
            last_Hrw_valid = false;  // Motion delta across a hard reset is not meaningful
        });

        on<Trigger<PenaltyReset>>().then([this](const PenaltyReset& reset) {
            log<INFO>("Resetting field localisation for penalty kick");
            Eigen::Vector3d penalty_state = reset.penalty_kick_position;

            // Mirror-flip detection: PenaltyReset is one of the few moments we get an external positional cue,
            // so use it to check whether our current estimate has likely flipped to the wrong half of the
            // field. This is a structural check only (see README for why a per-frame vision-based evidence
            // ratio is not implemented: the cost function is exactly mirror-symmetric).
            Eigen::Vector3d mirrored          = mirror_pose(filter.mean);
            double current_distance_to_reset  = (filter.mean.head<2>() - penalty_state.head<2>()).norm();
            double mirrored_distance_to_reset = (mirrored.head<2>() - penalty_state.head<2>()).norm();
            if (mirrored_distance_to_reset < current_distance_to_reset) {
                log<WARN>("Mirror flip detected via penalty reset (current estimate ",
                          current_distance_to_reset,
                          "m from penalty position, mirrored estimate ",
                          mirrored_distance_to_reset,
                          "m)");
                if (cfg.mirror_auto_flip) {
                    filter.mean = mirrored;
                }
            }

            state = penalty_state;
            filter.reset(state, cfg.change_limit);
            last_reset          = NUClear::clock::now();
            last_certain_state  = state;  // Update the last certain state
            last_Hrw_valid      = false;
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

                    // Predict step: grow the filter's covariance in proportion to the odometry motion since the
                    // last frame. The state (x,y,theta) parameterises Hfw, which is constant under perfect
                    // odometry, so there is no motion model beyond this noise injection.
                    double motion = 0.0;
                    if (last_Hrw_valid) {
                        Eigen::Isometry3d delta_Hrw = sensors.Hrw * last_Hrw.inverse();
                        double dtranslation         = delta_Hrw.translation().head<2>().norm();
                        double dyaw = std::abs(
                            utility::math::angle::normalise_angle(mat_to_rpy_intrinsic(delta_Hrw.rotation()).z()));
                        motion = dtranslation + dyaw;
                    }
                    filter.predict(motion, cfg.kalman_process_noise);
                    last_Hrw       = sensors.Hrw;
                    last_Hrw_valid = true;

                    double chosen_state_cost = 0.0;
                    Eigen::Vector3d proposed_state;

                    if (startup && cfg.starting_side == StartingSide::EITHER) {
                        // Find the best initial state to use based on the optimisation results of each
                        // hypothesis
                        std::vector<std::pair<Eigen::Vector3d, double>> opt_results{};
                        for (auto& hypothesis : cfg.initial_hypotheses) {
                            opt_results.push_back(
                                run_field_line_optimisation(hypothesis, field_lines.rPWw, field_intersections, goals));
                        }
                        auto best_hypothesis =
                            std::min_element(opt_results.begin(), opt_results.end(), [](const auto& a, const auto& b) {
                                return a.second < b.second;
                            });
                        proposed_state    = best_hypothesis->first;
                        chosen_state_cost = best_hypothesis->second;
                        // For startup, always accept the best hypothesis
                        state              = proposed_state;
                        last_certain_state = state;
                        filter.reset(state, cfg.change_limit);
                        startup = false;
                    }
                    else {
                        // Size the search box from the filter's current uncertainty: 3-sigma of (cov + a
                        // nominal representative measurement covariance), clamped per axis to
                        // [0.1, change_limit]. This box only bounds *this* optimisation call; the actual
                        // measurement covariance used for the filter update below is derived from this
                        // frame's Hessian, not this nominal value.
                        Eigen::Matrix3d nominal_R_for_bounds = Eigen::Matrix3d::Identity() * cfg.kalman_measurement_scale;
                        Eigen::Vector3d sigma3 = 3.0 * (filter.cov + nominal_R_for_bounds).diagonal().cwiseSqrt();
                        Eigen::Vector3d box;
                        for (int i = 0; i < 3; ++i) {
                            box(i) = std::clamp(sigma3(i), 0.1, cfg.change_limit(i));
                        }

                        // Run the optimisation routine, seeded from the filter's current mean
                        std::pair<Eigen::Vector3d, double> opt_results = run_field_line_optimisation(filter.mean,
                                                                                                      field_lines.rPWw,
                                                                                                      field_intersections,
                                                                                                      goals,
                                                                                                      false,
                                                                                                      box);
                        proposed_state    = opt_results.first;
                        chosen_state_cost = opt_results.second;

                        // Gate acceptance on the matched-percept validity fraction, not raw cost, since cost
                        // magnitudes are not directly comparable across differing numbers/mixes of
                        // observations.
                        double validity = compute_validity(proposed_state, field_lines.rPWw, field_intersections);
                        emit(graph("Validity", validity));

                        if (validity >= cfg.validity_min_validity) {
                            state = proposed_state;

                            // Derive a measurement covariance from the curvature of the cost function at the
                            // optimum, and fuse it into the filter.
                            Eigen::Vector3d h = cfg.kalman_hessian_step.cwiseMax(2.0 * cfg.grid_size);
                            Eigen::Matrix3d H =
                                finite_difference_hessian(proposed_state, field_lines.rPWw, field_intersections, goals, h);
                            double r_max = cfg.change_limit.squaredNorm();
                            double r_min = std::pow(2.0 * cfg.grid_size, 2);
                            Eigen::Matrix3d R =
                                covariance_from_hessian(H, cfg.kalman_measurement_scale, r_min, r_max);

                            filter.update(proposed_state, R);

                            // Update the last certain state and reset counter
                            last_certain_state = filter.mean;
                            num_invalid_frames = 0;
                        }
                        else {
                            // Reject the update, keep the current filter estimate (predict step already ran)
                            log<DEBUG>("Rejecting optimisation result: validity ",
                                       validity,
                                       " below threshold ",
                                       cfg.validity_min_validity);
                            num_invalid_frames++;
                        }

                        // Automatic local-minimum escape: the optimiser is local and re-converges to the same
                        // basin every frame, and a biased pose can keep validity above the accept threshold
                        // indefinitely (most points still land near *some* line). Once per window, probe the
                        // analytic reset candidates (global, association-free) unconditionally and jump only
                        // if one is decisively better and nearby. No stuckness detection is needed: when
                        // well-localised the best candidate matches the current pose and the improvement
                        // margin is never met, so the probe is a no-op.
                        recovery_validity_sum += validity;
                        recovery_frame_count++;
                        if (cfg.recovery_enabled && recovery_frame_count >= cfg.recovery_probe_period) {
                            const double mean_validity = recovery_validity_sum / recovery_frame_count;
                            recovery_validity_sum      = 0.0;
                            recovery_frame_count       = 0;
                            if ((NUClear::clock::now() - last_reset) > std::chrono::seconds(cfg.reset_delay)
                                && try_local_minimum_escape(field_lines, field_intersections, goals, mean_validity)) {
                                num_invalid_frames = 0;
                                last_reset         = NUClear::clock::now();
                            }
                        }
                    }

                    // Check if uncertainty is too high and trigger reset if needed
                    emit(graph("Cost", chosen_state_cost));
                    if (cfg.validity_reset_on_invalid && (num_invalid_frames > cfg.validity_max_invalid_frames)
                        && ((NUClear::clock::now() - last_reset) > std::chrono::seconds(cfg.reset_delay))) {
                        // Validity has been below threshold too many times, reset the localisation
                        log<WARN>("Validity below threshold ",
                                  cfg.validity_max_invalid_frames,
                                  " times, triggering uncertainty reset");
                        // Emit that we are resetting, eg for behaviour
                        emit(std::make_unique<UncertaintyResetFieldLocalisation>());
                        // Reset localisation by finding a new valid state
                        const auto reset_start = NUClear::clock::now();
                        uncertainty_reset(fd, field_lines, field_intersections, goals, sensors.Hrw);
                        const auto reset_end = NUClear::clock::now();
                        const auto reset_duration =
                            std::chrono::duration_cast<std::chrono::duration<double>>(reset_end - reset_start);
                        log<DEBUG>("Uncertainty reset duration (s): ", reset_duration.count());
                        // Reset variables
                        num_invalid_frames = 0;
                        last_reset          = NUClear::clock::now();
                        // Let other modules know that localisation has finished resetting
                        emit<Scope::DELAY>(std::make_unique<FinishReset>(), std::chrono::seconds(1));
                    }

                    // Emit the field message
                    auto field = std::make_unique<Field>();
                    field->Hfw = compute_Hfw(filter.mean);

                    // Debugging
                    if (log_level <= DEBUG) {
                        debug_field_localisation(field->Hfw);
                    }
                    // Association (run once for debugging in NUsight)
                    auto associations = data_association(field_intersections, field->Hfw);
                    for (const auto& association : associations) {
                        field->association_lines.push_back({association.first, association.second});
                    }

                    // Add cost, covariance, and uncertainty to the field message
                    field->cost        = chosen_state_cost;
                    field->covariance  = filter.cov;
                    field->uncertainty = filter.cov.trace();

                    emit(field);
                });
    }

    void FieldLocalisationNLopt::debug_field_localisation(Eigen::Isometry3d Hfw) {
        emit(graph("opt state", state.x(), state.y(), state.z()));
        emit(graph("filtered state", filter.mean.x(), filter.mean.y(), filter.mean.z()));
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

    double FieldLocalisationNLopt::evaluate_cost(const Eigen::Vector3d& x,
                                                 const std::vector<Eigen::Vector3d>& field_lines,
                                                 const std::shared_ptr<const FieldIntersections>& field_intersections,
                                                 const std::shared_ptr<const Goals>& goals) {
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

        return cost;
    }

    std::pair<Eigen::Vector3d, double> FieldLocalisationNLopt::run_field_line_optimisation(
        const Eigen::Vector3d& initial_guess,
        const std::vector<Eigen::Vector3d>& field_lines,
        const std::shared_ptr<const FieldIntersections>& field_intersections,
        const std::shared_ptr<const Goals>& goals,
        bool uncertainty_optimisation,
        const Eigen::Vector3d& box_bounds) {
        // Wrap the objective function in a lambda function. The state-change regulariser is added here, on
        // top of the perception-only evaluate_cost, so that the Hessian and validity metric (which both call
        // evaluate_cost directly) never see the artificial curvature it would inject in otherwise-unobservable
        // directions.
        ObjectiveFunction<double, 3> obj_fun =
            [&](const Eigen::Matrix<double, 3, 1>& x, Eigen::Matrix<double, 3, 1>& grad, void* data) -> double {
            (void) data;  // Unused in this case
            (void) grad;  // Unused in this case
            return evaluate_cost(x, field_lines, field_intersections, goals)
                   + cfg.state_change_weight * (x - initial_guess).squaredNorm();
        };
        // Create the NLopt optimizer and setup the algorithm, tolerances and maximum number of evaluations
        constexpr unsigned int n   = 3;
        nlopt::algorithm algorithm = uncertainty_optimisation ? cfg.uncertainty_algorithm : cfg.normal_algorithm;
        nlopt::opt opt             = nlopt::opt(algorithm, n);

        if (uncertainty_optimisation) {
            opt.set_xtol_rel(cfg.uncertainty_xtol_rel);
            opt.set_ftol_rel(cfg.uncertainty_ftol_rel);
            opt.set_maxeval(cfg.uncertainty_maxeval);
        }
        else {
            opt.set_xtol_rel(cfg.normal_xtol_rel);
            opt.set_ftol_rel(cfg.normal_ftol_rel);
            opt.set_maxeval(cfg.normal_maxeval);
        }

        // Set the objective function
        opt.set_min_objective(eigen_objective_wrapper<double, n>, &obj_fun);

        // Define the upper and lower bounds for the change in state. A zero box_bounds is the sentinel for
        // "use the configured change_limit" (the common case); callers with a covariance-derived box (see the
        // main loop) pass a non-zero per-axis bound instead.
        const Eigen::Vector3d& bounds = box_bounds.isZero() ? cfg.change_limit : box_bounds;
        Eigen::Vector3d lower_bounds  = initial_guess - bounds;
        Eigen::Vector3d upper_bounds  = initial_guess + bounds;

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
        nlopt::result result = opt.optimize(x, final_cost);

        log<DEBUG>("Ran optimisation with algorithm ",
                   algorithm,
                   " and initial guess (",
                   initial_guess.x(),
                   ", ",
                   initial_guess.y(),
                   ", ",
                   initial_guess.z(),
                   "), resulting in proposed state (",
                   x[0],
                   ", ",
                   x[1],
                   ", ",
                   x[2],
                   ") with cost ",
                   final_cost);

        // Debug information about the optimization result
        log<DEBUG>("Final Cost: ", final_cost);
        switch (result) {
            case nlopt::SUCCESS: log<DEBUG>("Optimization succeeded"); break;
            case nlopt::FTOL_REACHED: log<DEBUG>("Optimization stopped: function tolerance reached"); break;
            case nlopt::XTOL_REACHED: log<DEBUG>("Optimization stopped: variable tolerance reached"); break;
            case nlopt::MAXEVAL_REACHED:
                log<DEBUG>("Optimization stopped: maximum evaluations reached (ftol_rel=",
                           uncertainty_optimisation ? cfg.uncertainty_ftol_rel : cfg.normal_ftol_rel,
                           ", xtol_rel=",
                           uncertainty_optimisation ? cfg.uncertainty_xtol_rel : cfg.normal_xtol_rel,
                           ")");
                break;
            default: log<ERROR>("Optimization failed with code ", result); break;
        }
        log<DEBUG>("Number of evaluations: ", opt.get_numevals());

        // Convert the optimized solution back to an Eigen vector
        Eigen::Matrix<double, n, 1> optimized_solution(n);
        nlopt_to_eigen<double, n>(x, optimized_solution);
        return std::make_pair(optimized_solution, final_cost);
    }
}  // namespace module::localisation
