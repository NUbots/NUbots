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

// #include "message/support/FieldDescription.hpp"
// #include "message/vision/FieldIntersections.hpp"
// #include "message/vision/FieldLines.hpp"
// #include "message/vision/Goal.hpp"

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
        // Get robot position in field space
        Eigen::Isometry3d Hrf = Hrw * compute_Hfw(last_certain_state).inverse();
        Eigen::Vector3d rRFf  = Hrf.inverse().translation();

        // How much distance from the robot to each side of the field
        // This represents the allowed change in the robot and world
        double x_min = -(fd.dimensions.field_length / 2 + 0.2) - rRFf.x();
        double x_max = (fd.dimensions.field_length / 2 + 0.2) - rRFf.x();
        double y_min = -(fd.dimensions.field_width / 2 + 0.2) - rRFf.y();
        double y_max = (fd.dimensions.field_width / 2 + 0.2) - rRFf.y();

        // Define search window around last known good position
        const double window_size         = 2.0;
        const double step_size           = 0.3;
        const std::vector<double> angles = {0, M_PI_2, M_PI, -M_PI_2, M_PI_4, 3 * M_PI_4, -M_PI_4, -3 * M_PI_4};

        std::vector<std::pair<Eigen::Vector3d, double>> hypotheses;
        for (double dx = -window_size; dx <= window_size; dx += step_size) {
            for (double dy = -window_size; dy <= window_size; dy += step_size) {
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
                    Eigen::Vector3d hypothesis(x, y, angle);
                    double cost =
                        run_field_line_optimisation(hypothesis, field_lines.rPWw, field_intersections, goals).second;
                    hypotheses.emplace_back(hypothesis, cost);
                }
            }
        }

        // Sort hypotheses by cost (ascending)
        std::sort(hypotheses.begin(), hypotheses.end(), [](const auto& a, const auto& b) {
            return a.second < b.second;
        });

        // If local search is valid, use the lowest cost hypothesis
        if (!ranked_hypothesis.empty() && hypotheses[0].second < cfg.cost_threshold) {
            log<INFO>("Uncertainty reset (local): using best hypothesis", hypotheses[0].second);
            log<INFO>("Best hypothesis: ", hypotheses[0].second.transpose());

            // Set the state to the best hypothesis
            state = hypotheses[0].second;
            kf.set_state(state);
            kf.time(Eigen::Matrix<double, n_inputs, 1>::Zero(), 0);
            return;
        }

        // The local search did not yield a valid hypothesis, use a global search
        if (hypotheses.empty()) {
            log<INFO>("Uncertainty reset (global): No hypotheses found, searching whole field");
        }
        else {
            log<INFO>("Uncertainty reset (global): Cost too high, searching whole field", hypotheses[0].second);
        }

        // Iterate over the entire field area with a grid search
        hypotheses.clear();
        for (double x = x_min; x <= x_max; x += step_size) {
            for (double y = y_min; y <= y_max; y += step_size) {
                // Add hypotheses for each position with all compass and diagonal headings
                // Calculate cost using NLopt
                for (const auto& angle : angles) {
                    Eigen::Vector3d hypothesis(x, y, angle);
                    double cost =
                        run_field_line_optimisation(hypothesis, field_lines.rPWw, field_intersections, goals).second;
                    hypotheses.emplace_back(hypothesis, cost);
                }
            }
        }

        // Sort hypotheses by cost (ascending)
        std::sort(hypotheses.begin(), hypotheses.end(), [](const auto& a, const auto& b) {
            return a.second < b.second;
        });

        // If the best hypothesis is not on the same side, mirror it
        bool same_side = (hypotheses[0].first.x() * last_certain_state.x() >= 0);
        Eigen::Vector3d best_hypothesis =
            same_side ? hypotheses[0].first
                      : Eigen::Vector3d(-hypotheses[0].first.x(), -hypotheses[0].first.y(), hypotheses[0].first.z());
        log<INFO>("Same side: ", same_side, " (", hypotheses[0].first.x(), ", ", last_certain_state.x(), ")");

        log<INFO>("Best hypothesis: ", best_hypothesis.transpose());
        // Set the state to the best hypothesis
        state = best_hypothesis;
        kf.set_state(state);
        kf.time(Eigen::Matrix<double, n_inputs, 1>::Zero(), 0);
    }

}  // namespace module::localisation
