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

#include "FieldLocalisationNLopt.hpp"

#include "utility/math/angle.hpp"

namespace module::localisation {

    using message::support::FieldDescription;
    using message::vision::FieldIntersections;
    using message::vision::FieldLines;
    using message::vision::Goals;

    namespace {
        /// @brief Combined position + wrapped-angle distance between two poses, used for the mirror-twin
        /// tie-break (an explicit, documented tie-break rather than the implicit search-boundary one that
        /// constrain_to_half normally provides).
        double pose_distance(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
            double position_distance = (a.head<2>() - b.head<2>()).norm();
            double angle_distance    = std::abs(utility::math::angle::signedDifference(a.z(), b.z()));
            return position_distance + angle_distance;
        }

        /// @brief Whether pose b is approximately the mirror image of pose a (position within 0.5 m, heading
        /// within 0.3 rad of the exact mirror).
        bool is_mirror_twin(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
            Eigen::Vector3d mirrored = mirror_pose(a);
            return pose_distance(mirrored, b) < 0.8;
        }
    }  // namespace

    std::optional<std::pair<Eigen::Vector3d, double>> FieldLocalisationNLopt::candidate_probe(
        const FieldLines& field_lines,
        const std::shared_ptr<const FieldIntersections>& field_intersections,
        const std::shared_ptr<const Goals>& goals) {
        // Gather one-shot analytic candidates
        std::vector<Eigen::Vector3d> candidates;
        if (cfg.reset_use_candidates) {
            auto goal_candidates         = goal_pair_candidates(goals);
            auto intersection_candidates = intersection_pair_candidates(field_intersections);
            candidates.insert(candidates.end(), goal_candidates.begin(), goal_candidates.end());
            candidates.insert(candidates.end(), intersection_candidates.begin(), intersection_candidates.end());
        }
        candidates.push_back(last_certain_state);

        // When not constraining to the robot's current half, also consider the mirror image of every
        // candidate so far - mirror twins compete on validity like any other candidate.
        if (!cfg.reset_constrain_to_half) {
            std::vector<Eigen::Vector3d> mirrored;
            mirrored.reserve(candidates.size());
            for (const auto& candidate : candidates) {
                mirrored.push_back(mirror_pose(candidate));
            }
            candidates.insert(candidates.end(), mirrored.begin(), mirrored.end());
        }

        if (candidates.empty()) {
            return std::nullopt;
        }

        // Short SBPLX refine per candidate, keep the highest validity
        std::optional<std::pair<Eigen::Vector3d, double>> best;
        for (const auto& candidate : candidates) {
            auto [refined_state, cost] =
                run_field_line_optimisation(candidate, field_lines.rPWw, field_intersections, goals, true);
            (void) cost;
            double validity = compute_validity(refined_state, field_lines.rPWw, field_intersections);
            if (!best || validity > best->second) {
                best = std::make_pair(refined_state, validity);
            }
        }
        return best;
    }

    bool FieldLocalisationNLopt::try_local_minimum_escape(
        const FieldLines& field_lines,
        const std::shared_ptr<const FieldIntersections>& field_intersections,
        const std::shared_ptr<const Goals>& goals,
        double mean_validity) {
        auto best = candidate_probe(field_lines, field_intersections, goals);
        if (!best) {
            return false;
        }

        // Only escape to a decisively better pose that is local to the current estimate. The locality gate
        // (position + wrapped angle) also rejects mirror flips, which are always at least pi away in heading.
        bool decisively_better = best->second >= mean_validity + cfg.recovery_improvement_margin;
        bool local             = pose_distance(best->first, filter.mean) <= cfg.window_size;
        if (!decisively_better || !local) {
            return false;
        }

        log<INFO>("Local-minimum escape: candidate validity ",
                  best->second,
                  " beats window mean ",
                  mean_validity,
                  ", jumping from (",
                  filter.mean.transpose(),
                  ") to (",
                  best->first.transpose(),
                  ")");
        state = best->first;
        filter.reset(state, cfg.change_limit);
        last_certain_state = state;
        return true;
    }

    void FieldLocalisationNLopt::uncertainty_reset(const FieldDescription& fd,
                                                   const FieldLines& field_lines,
                                                   const std::shared_ptr<const FieldIntersections>& field_intersections,
                                                   const std::shared_ptr<const Goals>& goals,
                                                   const Eigen::Isometry3d& Hrw) {
        // --- Stage 1 + 2: analytic candidates, refined and ranked by validity ---
        if (auto best = candidate_probe(field_lines, field_intersections, goals)) {
            if (best->second >= cfg.validity_min_validity) {
                log<INFO>("Uncertainty reset (candidates): accepted with validity ", best->second);
                state = best->first;
                filter.reset(state, cfg.change_limit);
                last_certain_state = state;
                return;
            }
            log<INFO>("Uncertainty reset (candidates): best validity ",
                      best->second,
                      " below threshold ",
                      cfg.validity_min_validity,
                      ", falling back to grid search");
        }
        else {
            log<INFO>("Uncertainty reset (candidates): no candidates generated, falling back to grid search");
        }

        // --- Stage 3: grid-search fallback (local window around last_certain_state, then a wider search) ---
        // Get robot position in field space
        Eigen::Isometry3d Hrf = Hrw * compute_Hfw(last_certain_state).inverse();
        Eigen::Vector3d rRFf  = Hrf.inverse().translation();

        Eigen::Vector3d rRWf = compute_Hfw(last_certain_state).rotation() * Hrw.inverse().translation();

        // How much distance from the robot to each side of the field
        // This represents the allowed change in the robot and world
        double x_min = -(fd.dimensions.field_length / 2 + 0.2) + rRWf.x();
        double x_max = (fd.dimensions.field_length / 2 + 0.2) + rRWf.x();
        double y_min = -(fd.dimensions.field_width / 2 + 0.2) + rRWf.y();
        double y_max = (fd.dimensions.field_width / 2 + 0.2) + rRWf.y();

        // Handle the mirror field problem by halving the x boundary to the robot's current side. This also
        // avoids unnecessary computations. Only applied when reset.constrain_to_half is true (the default);
        // when false, the search covers the whole field and mirror twins are disambiguated by the tie-break
        // below instead of by never considering the other half.
        if (cfg.reset_constrain_to_half) {
            x_max = rRFf.x() < 0 ? rRWf.x() : x_max;
            x_min = rRFf.x() > 0 ? rRWf.x() : x_min;
        }

        std::vector<double> angles{};
        for (int i = 0; i < cfg.num_angles; ++i) {
            angles.push_back(i * (2 * M_PI / cfg.num_angles));  // Divide the full circle into equal parts
        }

        std::vector<std::pair<Eigen::Vector3d, double>> hypotheses;
        for (double dx = -cfg.window_size; dx <= cfg.window_size; dx += cfg.step_size) {
            for (double dy = -cfg.window_size; dy <= cfg.window_size; dy += cfg.step_size) {
                // Try hypotheses around the last certain state (state is world position)
                double x = last_certain_state.x() + dx;
                double y = last_certain_state.y() + dy;

                // Skip hypotheses outside the field boundaries
                if (x < x_min || x > x_max || y < y_min || y > y_max) {
                    continue;
                }

                // Add hypotheses for each position with all compass and diagonal headings
                // Calculate cost using NLopt
                for (const auto& angle : angles) {
                    hypotheses.emplace_back(run_field_line_optimisation(Eigen::Vector3d(x, y, angle),
                                                                        field_lines.rPWw,
                                                                        field_intersections,
                                                                        goals,
                                                                        true));
                }
            }
        }

        // Sort hypotheses by cost (ascending)
        std::sort(hypotheses.begin(), hypotheses.end(), [](const auto& a, const auto& b) {
            return a.second < b.second;
        });

        // Mirror-twin tie-break: if the best two hypotheses are mirror images of each other, prefer whichever
        // is closer (position + wrapped angle) to last_certain_state - an explicit, documented tie-break
        // instead of the implicit one that constrain_to_half normally provides by never searching the other
        // half at all.
        if (!cfg.reset_constrain_to_half && hypotheses.size() >= 2
            && is_mirror_twin(hypotheses[0].first, hypotheses[1].first)
            && pose_distance(hypotheses[1].first, last_certain_state)
                   < pose_distance(hypotheses[0].first, last_certain_state)) {
            std::swap(hypotheses[0], hypotheses[1]);
        }

        // If local search is valid, use the best hypothesis
        if (!hypotheses.empty()
            && compute_validity(hypotheses[0].first, field_lines.rPWw, field_intersections) >= cfg.validity_min_validity) {
            log<INFO>("Uncertainty reset (local): using best hypothesis, cost ", hypotheses[0].second);
            // Set the state to the best hypothesis
            state = hypotheses[0].first;
            filter.reset(state, cfg.change_limit);
            last_certain_state = state;  // Update the last certain state
            return;
        }

        // The local search did not yield a valid hypothesis, use a global search
        if (hypotheses.empty()) {
            log<INFO>("Uncertainty reset (global): No hypotheses found, searching whole field");
        }
        else {
            log<INFO>("Uncertainty reset (global): Validity too low, searching whole field");
        }

        // Iterate over the entire field area with a grid search
        hypotheses.clear();
        for (double x = x_min; x <= x_max; x += cfg.step_size) {
            for (double y = y_min; y <= y_max; y += cfg.step_size) {
                // Add hypotheses for each position with all compass and diagonal headings
                // Calculate cost using NLopt
                for (const auto& angle : angles) {
                    hypotheses.emplace_back(run_field_line_optimisation(Eigen::Vector3d(x, y, angle),
                                                                        field_lines.rPWw,
                                                                        field_intersections,
                                                                        goals,
                                                                        true));
                }
            }
        }

        // Sort hypotheses by cost (ascending)
        std::sort(hypotheses.begin(), hypotheses.end(), [](const auto& a, const auto& b) {
            return a.second < b.second;
        });

        // Mirror-twin tie-break, as above.
        if (!cfg.reset_constrain_to_half && hypotheses.size() >= 2
            && is_mirror_twin(hypotheses[0].first, hypotheses[1].first)
            && pose_distance(hypotheses[1].first, last_certain_state)
                   < pose_distance(hypotheses[0].first, last_certain_state)) {
            std::swap(hypotheses[0], hypotheses[1]);
        }

        // Set the state to the best hypothesis
        state = hypotheses[0].first;
        filter.reset(state, cfg.change_limit);
        last_certain_state = state;  // Update the last certain state
    }

}  // namespace module::localisation
