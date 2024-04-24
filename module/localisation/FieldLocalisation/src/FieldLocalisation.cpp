/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#include "FieldLocalisation.hpp"

#include <fstream>

#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/math/euler.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::localisation {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::localisation::Field;
    using message::localisation::ResetFieldLocalisation;
    using message::vision::FieldLines;

    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::nusight::graph;
    using utility::support::Expression;

    using namespace std::chrono;

    FieldLocalisation::FieldLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("FieldLocalisation.yaml").then([this](const Configuration& config) {
            this->log_level                   = config["log_level"].as<NUClear::LogLevel>();
            cfg.grid_size                     = config["grid_size"].as<double>();
            cfg.save_map                      = config["save_map"].as<bool>();
            cfg.starting_side                 = config["starting_side"].as<std::string>();
            cfg.start_time_delay              = std::chrono::seconds(config["start_time_delay"].as<int>());
            cfg.initial_state                 = Eigen::Vector3d(config["initial_state"].as<Expression>());
            cfg.use_ground_truth_localisation = config["use_ground_truth_localisation"].as<bool>();

            // Field line optimisation parameters
            cfg.field_line_distance_weight = config["field_line_distance_weight"].as<double>();
            cfg.min_field_line_points      = config["min_field_line_points"].as<int>();

            // Field line intersection optimisation parameters
            cfg.field_line_intersection_weight = config["field_line_intersection_weight"].as<double>();
            cfg.min_association_distance       = config["min_association_distance"].as<double>();
            cfg.min_field_line_intersections   = config["min_field_line_intersections"].as<int>();

            // Constraints and weighting on change in state
            cfg.change_limit        = Eigen::Vector3d(config["change_limit"].as<Expression>());
            cfg.state_change_weight = config["state_change_weight"].as<double>();

            // Optimisation parameters
            cfg.xtol_rel = config["opt"]["xtol_rel"].as<double>();
            cfg.ftol_rel = config["opt"]["ftol_rel"].as<double>();
            cfg.maxeval  = config["opt"]["maxeval"].as<int>();
        });

        on<Startup, Trigger<FieldDescription>>().then("Update Field Line Map", [this](const FieldDescription& fd) {
            // Generate the field line distance map
            setup_fieldline_distance_map(fd);
            // Generate the field landmarks
            setup_field_landmarks(fd);
            if (cfg.save_map) {
                std::ofstream file("recordings/fieldline_map.csv");
                file << fieldline_distance_map.get_map();
                file.close();
            }

            // Set the initial state as either left, right, both sides of the field or manually specified inital state
            auto left_side =
                Eigen::Vector3d((fd.dimensions.field_length / 4), (fd.dimensions.field_width / 2), -M_PI_2);
            auto right_side =
                Eigen::Vector3d((fd.dimensions.field_length / 4), (-fd.dimensions.field_width / 2), M_PI_2);
            switch (cfg.starting_side) {
                case StartingSide::LEFT: cfg.initial_hypotheses.emplace_back(left_side); break;
                case StartingSide::RIGHT: cfg.initial_hypotheses.emplace_back(right_side); break;
                case StartingSide::EITHER:
                    cfg.initial_hypotheses.emplace_back(left_side);
                    cfg.initial_hypotheses.emplace_back(right_side);
                    break;
                case StartingSide::CUSTOM: cfg.initial_hypotheses.emplace_back(cfg.initial_state); break;
                default: log<NUClear::ERROR>("Invalid starting_side specified"); break;
            }
            state = cfg.initial_hypotheses[0];

            main_loop_handle.disable();
            emit<Scope::DELAY>(std::make_unique<ResetFieldLocalisation>(), cfg.start_time_delay);
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
        });

        on<Trigger<ResetFieldLocalisation>>().then([this] {
            log<NUClear::INFO>("Resetting field localisation");
            main_loop_handle.enable();
            state   = cfg.initial_hypotheses[0];
            startup = true;
        });

        main_loop_handle = on<Trigger<FieldLines>, With<Stability>, With<RawSensors>, With<FieldIntersections>>().then(
            "NLopt field localisation",
            [this](const FieldLines& field_lines,
                   const Stability& stability,
                   const RawSensors& raw_sensors,
                   const FieldIntersections& field_intersections) {
                // Emit field message using ground truth if available
                if (cfg.use_ground_truth_localisation) {
                    auto field(std::make_unique<Field>());
                    field->Hfw = raw_sensors.localisation_ground_truth.Hfw;
                    emit(field);
                    return;
                }

                // Otherwise calculate using field line distance map
                bool unstable = stability == Stability::FALLEN || stability == Stability::FALLING;
                if (!unstable && field_lines.rPWw.size() > cfg.min_field_line_points) {
                    if (startup && cfg.starting_side == StartingSide::EITHER) {
                        std::vector<std::pair<Eigen::Vector3d, double>> opt_results;
                        for (auto& hypothesis : cfg.initial_hypotheses) {
                            opt_results.push_back(
                                run_field_line_optimisation(hypothesis, field_lines.rPWw, field_intersections));
                        }
                        // Find the best initial state to use based on the optimisation results of each hypothesis
                        auto best_hypothesis =
                            std::min_element(opt_results.begin(), opt_results.end(), [](const auto& a, const auto& b) {
                                return a.second < b.second;
                            });
                        state = best_hypothesis->first;
                        log<NUClear::DEBUG>("Initial state optimisation took ",
                                            best_hypothesis->first.transpose(),
                                            " with cost ",
                                            best_hypothesis->second);
                        startup = false;
                    }
                    else {
                        // Run optimisation routine
                        std::pair<Eigen::Vector3d, double> opt_results =
                            run_field_line_optimisation(state, field_lines.rPWw, field_intersections);
                        state = opt_results.first;
                    }
                    auto field(std::make_unique<Field>());
                    field->Hfw = compute_Hfw(state);
                    if (log_level <= NUClear::DEBUG && raw_sensors.localisation_ground_truth.exists) {
                        debug_field_localisation(field->Hfw, raw_sensors);
                    }
                    emit(field);
                }
            });
    }

    void FieldLocalisation::debug_field_localisation(Eigen::Isometry3d Hfw, const RawSensors& raw_sensors) {
        const Eigen::Isometry3d true_Hfw = Eigen::Isometry3d(raw_sensors.localisation_ground_truth.Hfw);
        // Determine translational distance error
        Eigen::Vector3d true_rFWw  = true_Hfw.translation();
        Eigen::Vector3d rFWw       = (Hfw.translation());
        Eigen::Vector3d error_rFWw = (true_rFWw - rFWw).cwiseAbs();
        // Determine yaw, pitch, and roll error
        Eigen::Vector3d true_Rfw  = mat_to_rpy_intrinsic(true_Hfw.rotation());
        Eigen::Vector3d Rfw       = mat_to_rpy_intrinsic(Hfw.rotation());
        Eigen::Vector3d error_Rfw = (true_Rfw - Rfw).cwiseAbs();
        double quat_rot_error     = Eigen::Quaterniond(true_Hfw.linear() * Hfw.inverse().linear()).w();

        // Graph translation and error from ground truth
        emit(graph("Hfw true translation (rFWw)", true_rFWw.x(), true_rFWw.y(), true_rFWw.z()));
        emit(graph("Hfw translation error", error_rFWw.x(), error_rFWw.y(), error_rFWw.z()));
        // Graph rotation and error from ground truth
        emit(graph("Rfw true angles (rpy)", true_Rfw.x(), true_Rfw.y(), true_Rfw.z()));
        emit(graph("Rfw error (rpy)", error_Rfw.x(), error_Rfw.y(), error_Rfw.z()));
        emit(graph("Rfw quaternion rotational error", quat_rot_error));
    }

    Eigen::Isometry3d FieldLocalisation::compute_Hfw(const Eigen::Vector3d& state) {
        return Eigen::Translation<double, 3>(state.x(), state.y(), 0)
               * Eigen::AngleAxis<double>(state.z(), Eigen::Matrix<double, 3, 1>::UnitZ());
    }

    Eigen::Vector2i FieldLocalisation::position_in_map(const Eigen::Vector3d& particle, const Eigen::Vector3d& rPWw) {
        // Transform observations from world {w} to field {f} space
        Eigen::Vector3d rPFf = compute_Hfw(particle) * rPWw;

        // Get the associated index in the field line map [x, y]
        // Note: field space is placed in centre of the field, whereas the  map origin (x,y) is top left corner
        return Eigen::Vector2i(fieldline_distance_map.get_length() / 2 - std::round(rPFf(1) / cfg.grid_size),
                               fieldline_distance_map.get_width() / 2 + std::round(rPFf(0) / cfg.grid_size));
    }

    std::pair<Eigen::Vector3d, double> FieldLocalisation::run_field_line_optimisation(
        const Eigen::Vector3d& initial_guess,
        const std::vector<Eigen::Vector3d>& observations,
        const FieldIntersections& field_intersections) {

        // Wrap the objective function in a lambda function
        ObjectiveFunction<double, 3> obj_fun =
            [&](const Eigen::Matrix<double, 3, 1>& x, Eigen::Matrix<double, 3, 1>& grad, void* data) -> double {
            (void) data;  // Unused in this case
            (void) grad;  // Unused in this case

            // Compute the cost and gradient
            double field_line_point_cost = 0.0;
            for (auto rORr : observations) {
                // Get the position [x, y] of the observation in the map for this particle
                Eigen::Vector2i map_position = position_in_map(x, rORr);
                field_line_point_cost +=
                    std::pow(fieldline_distance_map.get_occupancy_value(map_position.x(), map_position.y()), 2);
            }
            field_line_point_cost = cfg.field_line_distance_weight * field_line_point_cost;

            // Compute the cost and gradient
            auto Hfw                            = compute_Hfw(x);
            double field_line_intersection_cost = 0.0;

            // Data association
            for (const auto& intersection : field_intersections.intersections) {
                double min_distance = std::numeric_limits<double>::max();

                for (const auto& landmark : landmarks) {
                    // Check if the landmark is of the same type as the observed intersection
                    if (landmark.type == intersection.type) {
                        // Calculate Euclidean distance between the observed intersection and the landmark
                        double distance = (landmark.rLFf - Hfw * intersection.rIWw).norm();

                        // If this landmark is closer update
                        if (distance < min_distance) {
                            min_distance = distance;
                        }
                    }
                }

                // If the closest landmark is too far away, do not consider it as an association
                if (min_distance < cfg.min_association_distance) {
                    field_line_intersection_cost += std::pow(min_distance, 2);
                }
            }
            field_line_intersection_cost = cfg.field_line_intersection_weight * field_line_intersection_cost;

            double state_change_cost = cfg.state_change_weight * (x - initial_guess).squaredNorm();
            double cost              = field_line_point_cost + field_line_intersection_cost + state_change_cost;
            emit(graph("Cost", cost));
            emit(graph("Field line point cost", field_line_point_cost));
            emit(graph("Field line intersection cost", field_line_intersection_cost));
            emit(graph("State change cost", state_change_cost));
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

        // Convert the optimized solution back to an Eigen
        Eigen::Matrix<double, n, 1> optimized_solution(n);
        nlopt_to_eigen<double, n>(x, optimized_solution);
        return std::make_pair(optimized_solution, final_cost);
    }
}  // namespace module::localisation
