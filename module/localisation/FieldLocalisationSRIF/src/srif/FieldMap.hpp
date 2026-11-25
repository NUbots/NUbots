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

/**
 * @file FieldMap.hpp
 * @brief Defines a static map of known landmark positions on a RoboCup humanoid soccer field
 *
 * @section field_frame Field coordinate frame {f}
 * The field frame {f} is a right-handed frame with:
 *  - Origin at the centre of the field
 *  - +x axis directed along the long axis of the field
 *  - +y axis directed along the short axis of the field
 *  - +z axis normal to the field surface, pointing up out of the ground (right-handed frame);
 *    all landmarks in this file lie on the ground plane, so z = 0 for every landmark.
 *
 */

#ifndef MODULE_LOCALISATION_SRIF_FIELDMAP_HPP
#define MODULE_LOCALISATION_SRIF_FIELDMAP_HPP

#include <Eigen/Core>
#include <vector>

namespace module::localisation::srif {

    /**
     * @brief Physical measurements of a RoboCup humanoid soccer field, in metres
     *
     * Field names and defaults mirror
     * NUbots/module/support/configuration/SoccerConfig/data/config/FieldDescription.yaml and
     * NUbots/shared/message/support/FieldDescription.proto (message FieldDescription.FieldDimensions).
     */
    struct FieldDimensions {
        // Values come from FieldDescription.yaml
        double line_width;              ///< Width of field lines
        double field_length;            ///< Touchline (sideline) length
        double field_width;             ///< Goal line (baseline) length
        double goal_depth;              ///< Distance behind the goal line to the back of the net
        double goal_width;              ///< Distance between the inner edges of the goal posts
        double goal_area_length;        ///< Goal area (6-yard box) length, from the goal line
        double goal_area_width;         ///< Goal area (6-yard box) width
        double penalty_mark_distance;   ///< Distance from the goal line to the penalty mark
        double centre_circle_diameter;  ///< Diameter of the centre circle
        double penalty_area_length;     ///< Penalty area (18-yard box) length, from the goal line
        double penalty_area_width;      ///< Penalty area (18-yard box) width
        double goalpost_width;          ///< Diameter of a (circular) goal post
        double border_strip_min_width;  ///< Minimum width of the border strip around the field
    };

    /**
     * @brief Classification of a field landmark
     */
    enum class LandmarkType { L_INTERSECTION, T_INTERSECTION, X_INTERSECTION, GOAL_POST };

    /**
     * @brief A static map of known field landmark positions in the field frame {f}
     *
     * See @ref field_frame "the file-level documentation" for the field frame convention.
     */
    class FieldMap {
    public:
        /**
         * @brief Construct the field map from a set of field dimensions
         * @param dims Field dimensions to build the landmark map from (see srif::field_dimensions)
         */
        explicit FieldMap(const FieldDimensions& dims);

        /**
         * @brief A painted line segment on the field (centreline coordinates, ground plane)
         */
        struct LineSegment {
            Eigen::Vector2d a;  ///< Segment start (x, y) in {f}
            Eigen::Vector2d b;  ///< Segment end (x, y) in {f}
        };

        /**
         * @brief A painted circle on the field (centreline coordinates, ground plane)
         */
        struct Circle {
            Eigen::Vector2d centre;  ///< Circle centre (x, y) in {f}
            double radius;           ///< Circle radius
        };

        /**
         * @brief Get all known landmark positions of a given type
         * @param type Landmark type to retrieve
         * @return Landmark positions rLFf (landmark relative to field origin, expressed in the
         *         field frame {f}), each with z = 0 (ground plane)
         */
        const std::vector<Eigen::Vector3d>& landmarks(LandmarkType type) const;

        /**
         * @brief Squared distance from a ground-plane point to the nearest painted line.
         *
         * Analytic minimum over all line segments and circles. Working with the
         * squared distance keeps the function smooth through zero (points exactly
         * on a line), which matters for derivative-based optimisation.
         *
         * @param p Point (x, y) in the field frame {f}
         * @return Squared distance to the nearest line centreline [m^2]
         */
        template <typename Scalar>
        Scalar distance_squared_to_nearest_line(const Eigen::Vector2<Scalar>& p) const {
            using std::sqrt;
            Scalar best = Scalar(1e12);
            for (const LineSegment& seg : line_segments_) {
                const Eigen::Vector2<Scalar> a  = seg.a.cast<Scalar>();
                const Eigen::Vector2<Scalar> ab = (seg.b - seg.a).cast<Scalar>();
                const double len2               = (seg.b - seg.a).squaredNorm();
                Scalar t                        = (p - a).dot(ab) / Scalar(len2);
                if (t < Scalar(0))
                    t = Scalar(0);
                if (t > Scalar(1))
                    t = Scalar(1);
                Scalar d2 = (p - (a + t * ab)).squaredNorm();
                if (d2 < best)
                    best = d2;
            }
            for (const Circle& c : line_circles_) {
                const Eigen::Vector2<Scalar> pc = p - c.centre.cast<Scalar>();
                Scalar n2                       = pc.squaredNorm();
                if (n2 < Scalar(1e-12))
                    n2 = Scalar(1e-12);  // Guard sqrt at the circle centre
                Scalar n  = sqrt(n2);
                Scalar d  = n - Scalar(c.radius);
                Scalar d2 = d * d;
                if (d2 < best)
                    best = d2;
            }
            return best;
        }

        FieldDimensions dims;  ///< Field dimensions used to build this map

    private:
        /// @brief Populate the landmarks and line primitives from #dims
        void build();

        std::vector<Eigen::Vector3d> landmarks_l_;          ///< L-intersection landmarks rLFf
        std::vector<Eigen::Vector3d> landmarks_t_;          ///< T-intersection landmarks rLFf
        std::vector<Eigen::Vector3d> landmarks_x_;          ///< X-intersection landmarks rLFf
        std::vector<Eigen::Vector3d> landmarks_goal_post_;  ///< Goal post landmarks rLFf
        std::vector<LineSegment> line_segments_;            ///< Painted line segments
        std::vector<Circle> line_circles_;                  ///< Painted circles
    };
}  // namespace module::localisation::srif

#endif  // MODULE_LOCALISATION_SRIF_FIELDMAP_HPP
