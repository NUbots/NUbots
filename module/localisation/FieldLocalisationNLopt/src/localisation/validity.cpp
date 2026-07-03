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

namespace module::localisation {

    using message::vision::FieldIntersections;

    double FieldLocalisationNLopt::compute_validity(
        const Eigen::Vector3d& state,
        const std::vector<Eigen::Vector3d>& field_lines,
        const std::shared_ptr<const FieldIntersections>& field_intersections) {

        // Fraction of field-line points that land within line_inlier_distance of the field-line map.
        double line_validity = 0.0;
        if (!field_lines.empty()) {
            size_t inliers = 0;
            for (const auto& rPWw : field_lines) {
                Eigen::Vector2i map_position = position_in_map(state, rPWw);
                double distance = fieldline_distance_map.get_occupancy_value(map_position.x(), map_position.y());
                if (distance >= 0.0 && distance < cfg.validity_line_inlier_distance) {
                    ++inliers;
                }
            }
            line_validity = static_cast<double>(inliers) / static_cast<double>(field_lines.size());
        }

        // No intersections observed: fall back to a lines-only validity.
        if (!field_intersections || field_intersections->intersections.empty()) {
            return line_validity;
        }

        // Fraction of observed intersections successfully associated with a landmark (data_association()
        // already rejects associations beyond max_association_distance).
        Eigen::Isometry3d Hfw = compute_Hfw(state);
        auto associations     = data_association(field_intersections, Hfw);
        double intersection_validity =
            static_cast<double>(associations.size()) / static_cast<double>(field_intersections->intersections.size());

        // Blend 50/50 when both modalities are present; lines-only if there were no field line points at all.
        return field_lines.empty() ? intersection_validity : 0.5 * line_validity + 0.5 * intersection_validity;
    }

}  // namespace module::localisation
