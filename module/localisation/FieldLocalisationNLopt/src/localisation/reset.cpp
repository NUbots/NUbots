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
                                                   const std::shared_ptr<const Goals>& goals) {
        // Define search window around last known good position
        const double window_size = 2.0;
        const double step_size   = 0.5;

        std::vector<Eigen::Vector3d> hypotheses;
        log<INFO>("Good value", last_certain_state.transpose());
        for (double dx = -window_size; dx <= window_size; dx += step_size) {
            for (double dy = -window_size; dy <= window_size; dy += step_size) {
                double x = last_certain_state.x() + dx;
                double y = last_certain_state.y() + dy;
                log<INFO>("Hypothesis position:", x, y);
                hypotheses.emplace_back(x, y, 0);
                hypotheses.emplace_back(x, y, M_PI_2);
                hypotheses.emplace_back(x, y, M_PI);
                hypotheses.emplace_back(x, y, -M_PI_2);

                hypotheses.emplace_back(x, y, M_PI_4);
                hypotheses.emplace_back(x, y, 3 * M_PI_4);
                hypotheses.emplace_back(x, y, -M_PI_4);
                hypotheses.emplace_back(x, y, -3 * M_PI_4);
            }
        }

        // Evaluate all hypotheses using NLopt
        std::vector<std::pair<Eigen::Vector3d, double>> ranked_hypotheses;
        for (const auto& hypothesis : hypotheses) {
            double cost = run_field_line_optimisation(hypothesis, field_lines.rPWw, field_intersections, goals).second;
            ranked_hypotheses.emplace_back(hypothesis, cost);
        }

        // Sort hypotheses by cost (ascending)
        std::sort(ranked_hypotheses.begin(), ranked_hypotheses.end(), [](const auto& a, const auto& b) {
            return a.second < b.second;
        });

        const Eigen::Vector3d& best_hypothesis = ranked_hypotheses[0].first;

        // No mirror ambiguity check needed — search is local to last known good state
        state = best_hypothesis;
        kf.set_state(state);
        kf.time(Eigen::Matrix<double, n_inputs, 1>::Zero(), 0);

        log<INFO>("Uncertainty reset (local): Chosen hypothesis:", state.transpose());
    }

}  // namespace module::localisation
