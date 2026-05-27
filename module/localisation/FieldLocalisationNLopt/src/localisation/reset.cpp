/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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

    void FieldLocalisationNLopt::uncertainty_reset(const FieldDescription& fd,
                                                   const FieldLines& field_lines,
                                                   const std::shared_ptr<const FieldIntersections>& field_intersections,
                                                   const std::shared_ptr<const Goals>& goals,
                                                   const Eigen::Isometry3d& Hrw) {

        log<DEBUG>("Starting uncertainty reset with local search around last certain state at",
                   last_certain_state.transpose());

        // Get robot position in field space
        Eigen::Isometry3d Hrf = Hrw * compute_Hfw(last_certain_state).inverse();
        Eigen::Vector3d rRFf  = Hrf.inverse().translation();

        Eigen::Vector3d rRWf = compute_Hfw(last_certain_state).rotation() * Hrw.inverse().translation();

        log<DEBUG>("Step 1");

        // How much distance from the robot to each side of the field
        // This represents the allowed change in the robot and world
        double x_min = -(fd.dimensions.field_length / 2 + 0.2) + rRWf.x();
        double x_max = (fd.dimensions.field_length / 2 + 0.2) + rRWf.x();
        double y_min = -(fd.dimensions.field_width / 2 + 0.2) + rRWf.y();
        double y_max = (fd.dimensions.field_width / 2 + 0.2) + rRWf.y();

        log<DEBUG>("Step 2");

        // Handle the mirror field problem by halving the x boundary to the robot's current side
        // This also avoids unnecessary computations
        x_max = rRFf.x() < 0 ? rRWf.x() : x_max;
        x_min = rRFf.x() > 0 ? rRWf.x() : x_min;

        std::vector<double> angles{};
        for (int i = 0; i < cfg.num_angles; ++i) {
            angles.push_back(i * (2 * M_PI / cfg.num_angles));  // Divide the full circle into equal parts
        }

        log<DEBUG>("Step 3");

        std::vector<std::pair<Eigen::Vector3d, double>> hypotheses;

        log<DEBUG>("Step 4");

        // Debug counters and estimates
        long long hypothesis_count = 0;
        int nx                     = static_cast<int>((cfg.window_size * 2) / cfg.step_size) + 1;
        long long estimated_total  = static_cast<long long>(nx) * nx * static_cast<long long>(angles.size());
        log<DEBUG>("Local search: window_size, step_size, num_angles, estimated_total",
                   cfg.window_size,
                   cfg.step_size,
                   angles.size(),
                   estimated_total);

        for (double dx = -cfg.window_size; dx <= cfg.window_size; dx += cfg.step_size) {
            log<DEBUG>("Local search: dx start", dx);  // coarse progress marker
            for (double dy = -cfg.window_size; dy <= cfg.window_size; dy += cfg.step_size) {
                // Try hypotheses around the last certain state (state is world position)
                double x = last_certain_state.x() + dx;
                double y = last_certain_state.y() + dy;

                // Skip hypotheses outside the field boundaries
                if (x < x_min || x > x_max || y < y_min || y > y_max) {
                    // log occasionally to avoid excessive spam
                    if ((++hypothesis_count & 0x3FF) == 0) {  // every 1024 skips/tries
                        log<DEBUG>("Local search: skipping outside field", x, y, x_min, x_max, y_min, y_max);
                    }
                    continue;
                }

                // Add hypotheses for each position with all compass and diagonal headings
                // Calculate cost using NLopt
                int start_time = NUClear::clock::now().time_since_epoch().count();
                for (const auto& angle : angles) {
                    log<DEBUG>("Starting field line optimisation for hypothesis", x, y, angle);
                    int start_of_field_line_opt = NUClear::clock::now().time_since_epoch().count();
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

        log<DEBUG>("Step 6");

        // If local search is valid, use the lowest cost hypothesis
        if (!hypotheses.empty() && hypotheses[0].second < cfg.cost_threshold) {
            log<INFO>("Uncertainty reset (local): using best hypothesis", hypotheses[0].second);
            // Set the state to the best hypothesis
            state              = hypotheses[0].first;
            filtered_state     = state;
            first_measurement  = true;
            last_certain_state = state;  // Update the last certain state
            return;
        }

        log<DEBUG>("Step 7");

        // The local search did not yield a valid hypothesis, use a global search
        if (hypotheses.empty()) {
            log<INFO>("Uncertainty reset (global): No hypotheses found, searching whole field");
        }
        else {
            log<INFO>("Uncertainty reset (global): Cost too high, searching whole field", hypotheses[0].second);
        }

        log<DEBUG>("Step 8");

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

        log<DEBUG>("Step 9");

        // Sort hypotheses by cost (ascending)
        std::sort(hypotheses.begin(), hypotheses.end(), [](const auto& a, const auto& b) {
            return a.second < b.second;
        });

        log<DEBUG>("Step 10");

        // Set the state to the best hypothesis
        state              = hypotheses[0].first;
        filtered_state     = state;
        first_measurement  = true;
        last_certain_state = state;  // Update the last certain state
    }

}  // namespace module::localisation
